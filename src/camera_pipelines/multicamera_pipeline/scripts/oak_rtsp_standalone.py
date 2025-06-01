#!/usr/bin/env python3
"""
Minimal RTSP streamer for one Luxonis OAK camera
(no ROS, no GStreamer CLI).  Launch one copy per camera:

    python3 oak_rtsp_standalone.py --mxid 18443010C142E7F400 --port 8554 --mount front_camera
    python3 oak_rtsp_standalone.py --mxid 18443010D1DEE7F400  --port 8555 --mount fork_camera

Streams appear at   rtsp://<this-host>:8554/<mount>
"""

import argparse
import queue
import signal
import sys
import threading
import logging
import time
import os
import resource

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Set real-time priority (commented out due to permission issues)
# try:
#     # Set nice value
#     os.nice(-20)
#
#     # Set resource limits
#     resource.setrlimit(resource.RLIMIT_RTPRIO, (99, 99))
#     resource.setrlimit(resource.RLIMIT_MEMLOCK, (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
#
#     logger.info("Real-time scheduling enabled")
# except Exception as e:
#     logger.warning(f"Could not set real-time priority: {e}")

# Performance monitoring
class PerformanceMonitor:
    def __init__(self):
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0
        self.latency = 0
        self.max_latency = 0
        self.latency_samples = []
        self.latency_sample_size = 100

    def update(self, frame_time):
        current_time = time.time()
        self.frame_count += 1
        if current_time - self.last_time >= 1:
            self.fps = self.frame_count
            self.frame_count = 0
            self.last_time = current_time

            # Calculate latency
            latency = (current_time - frame_time) * 1000  # ms
            self.latency = latency

            # Track max latency
            self.max_latency = max(self.max_latency, latency)

            # Track latency distribution
            self.latency_samples.append(latency)
            if len(self.latency_samples) > self.latency_sample_size:
                self.latency_samples.pop(0)

            # Calculate average latency
            avg_latency = sum(self.latency_samples) / len(self.latency_samples)

            logger.info(f"FPS: {self.fps}, Avg Latency: {avg_latency:.1f}ms, Max Latency: {self.max_latency:.1f}ms")

monitor = PerformanceMonitor()

import depthai as dai
import gi

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GObject  # noqa: E402

Gst.init(None)


# --------------------------------------------------------------------------- #
#  DepthAI: build a **video-only** pipeline (1080 p @ 30 FPS → H-265)         #
# --------------------------------------------------------------------------- #
def make_pipeline(fps: int = 30) -> dai.Pipeline:
    pipe = dai.Pipeline()

    # Configure camera for ultra-low latency
    cam = pipe.createColorCamera()
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(fps)
    cam.setPreviewKeepAspectRatio(False)
    cam.setInterleaved(False)
    cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam.setPreviewSize(1920, 1080)  # Full resolution preview
    cam.setIspScale(1, 1)  # No scaling
    cam.setNumFramesPool(2, 2, 2, 2, 2)  # Minimum pool size for ultra-low latency
    cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)  # Use CAM_A instead of RGB

    # Configure encoder for low latency
    enc = pipe.createVideoEncoder()
    enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
    enc.setRateControlMode(dai.VideoEncoderProperties.RateControlMode.CBR)  # Correct method name
    enc.setBitrate(4_000_000)  # 4 Mbps
    enc.setKeyframeFrequency(fps)  # One keyframe per second
    enc.setNumBFrames(0)  # No B-frames for lower latency
    enc.setFrameRate(fps)
    enc.setNumFramesPool(4)  # Small pool for low latency

    # Configure XLink for ultra-low latency
    xout = pipe.createXLinkOut()
    xout.setStreamName("h265")
    xout.input.setBlocking(False)
    xout.input.setQueueSize(1)  # Minimum queue size for ultra-low latency

    cam.video.link(enc.input)
    enc.bitstream.link(xout.input)

    return pipe


# --------------------------------------------------------------------------- #
#  RTSP: one mount-point per script                                           #
# --------------------------------------------------------------------------- #
def spawn_rtsp_server(port: int, mount: str) -> GstRtspServer.RTSPMediaFactory:
    server = GstRtspServer.RTSPServer()
    server.set_service(str(port))

    factory = GstRtspServer.RTSPMediaFactory()
    factory.set_launch(
        "( appsrc name=src is-live=true do-timestamp=true format=time "
        "! queue max-size-buffers=1 max-size-time=0 leaky=downstream "
        "! h265parse "
        "! rtph265pay config-interval=1 pt=96 name=pay0 )"
    )
    factory.set_shared(True)
    server.get_mount_points().add_factory(f"/{mount}", factory)
    server.attach(None)
    return factory


def main() -> None:
    ap = argparse.ArgumentParser()
    # ap.add_argument("--ip", help="PoE IP or MXID (USB).  Omit for first detected device.")
    ap.add_argument("--port", default=8554, type=int, help="RTSP TCP port")
    ap.add_argument("--mxid", help="MXID (more reliable than IP).  Omit for first detected device.")
    ap.add_argument("--mount", default="oak", help="RTSP mount point (eg. front_camera)")
    ap.add_argument("--fps", default=30, type=int, help="Camera frame-rate")
    ap.add_argument("--buffer-size", default=1, type=int, help="Output queue buffer size")
    args = ap.parse_args()

    # device_info = dai.DeviceInfo(args.ip) if args.ip else None
    device_info = dai.DeviceInfo(args.mxid) if args.mxid else None
    pipe = make_pipeline(args.fps)

    # ----------- Start DepthAI device -------------------------------------- #
    try:
        dev = dai.Device(pipe, device_info)
        logger.info(f"Connected to device: {dev.getDeviceName()}")
    except RuntimeError as exc:
        logger.error(f"Failed to connect to OAK device: {exc}")
        sys.exit(1)

    q = dev.getOutputQueue("h265", maxSize=args.buffer_size, blocking=False)

    # ----------- Start RTSP server ----------------------------------------- #
    factory = spawn_rtsp_server(args.port, args.mount)
    logger.info(f"RTSP stream ready at rtsp://<this-host>:{args.port}/{args.mount}")

    # Background thread feeds Gst‐appsrc with H-265 byte-streams ------------- #
    def pump_frames() -> None:
        appsrc = None
        caps_ready = threading.Event()
        last_frame_time = time.time()

        def on_configure(_factory, media):
            nonlocal appsrc
            appsrc = media.get_element().get_child_by_name("src")
            caps = Gst.Caps.from_string("video/x-h265,stream-format=(string)byte-stream")
            appsrc.set_property("caps", caps)
            caps_ready.set()

        factory.connect("media-configure", on_configure)

        while True:
            frame_time = time.time()
            frame = q.tryGet()  # non-blocking

            if not frame or not caps_ready.is_set():
                continue

            if appsrc:
                try:
                    data = frame.getData().tobytes()
                    buf = Gst.Buffer.new_allocate(None, len(data), None)
                    buf.fill(0, data)
                    appsrc.emit("push-buffer", buf)
                    monitor.update(frame_time)
                except Exception as e:
                    logger.error(f"Error processing frame: {e}")
                    continue

    # Start frame pump thread with real-time priority
    frame_thread = threading.Thread(target=pump_frames, daemon=True)
    frame_thread.start()

    # Graceful shutdown on Ctrl-C
    def shutdown_handler(signum, frame):
        logger.info("Shutting down...")
        loop.quit()
        frame_thread.join(timeout=2.0)
        if frame_thread.is_alive():
            logger.warning("Frame thread did not terminate cleanly")
        sys.exit(0)

    loop = GObject.MainLoop()
    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        loop.run()
    except KeyboardInterrupt:
        shutdown_handler(signal.SIGINT, None)
    except Exception as e:
        logger.error(f"Main loop error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()

