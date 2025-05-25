#!/usr/bin/env python3
"""
Minimal RTSP streamer for one Luxonis OAK camera
(no ROS, no GStreamer CLI).  Launch one copy per camera:

    python3 oak_rtsp_standalone.py --ip 192.168.50.10 --mount front_camera
    python3 oak_rtsp_standalone.py --ip 192.168.50.11 --mount fork_camera

Streams appear at   rtsp://<this-host>:8554/<mount>
"""

import argparse
import queue
import signal
import sys
import threading

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

    cam = pipe.createColorCamera()
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setFps(fps)
    cam.setPreviewKeepAspectRatio(False)

    enc = pipe.createVideoEncoder()
    enc.setDefaultProfilePreset(fps, dai.VideoEncoderProperties.Profile.H265_MAIN)
    cam.video.link(enc.input)

    xout = pipe.createXLinkOut()
    xout.setStreamName("h265")
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
        "! h265parse ! rtph265pay config-interval=1 name=pay0 pt=96 )"
    )
    factory.set_shared(True)
    server.get_mount_points().add_factory(f"/{mount}", factory)
    server.attach(None)
    return factory


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", help="PoE IP or MXID (USB).  Omit for first detected device.")
    ap.add_argument("--port", default=8554, type=int, help="RTSP TCP port")
    ap.add_argument("--mount", default="oak", help="RTSP mount point (eg. front_camera)")
    ap.add_argument("--fps", default=30, type=int, help="Camera frame-rate")
    args = ap.parse_args()

    device_info = dai.DeviceInfo(args.ip) if args.ip else None
    pipe = make_pipeline(args.fps)

    # ----------- Start DepthAI device -------------------------------------- #
    try:
        dev = dai.Device(pipe, device_info)
    except RuntimeError as exc:
        sys.exit(f"Failed to connect to OAK device ({exc})")

    q = dev.getOutputQueue("h265", maxSize=30, blocking=False)

    # ----------- Start RTSP server ----------------------------------------- #
    factory = spawn_rtsp_server(args.port, args.mount)
    print(f"RTSP stream ready  ➜  rtsp://<this-host>:{args.port}/{args.mount}")

    # Background thread feeds Gst‐appsrc with H-265 byte-streams ------------- #
    def pump_frames() -> None:
        appsrc = None

        def on_configure(_factory, media):
            nonlocal appsrc
            appsrc = media.get_element().get_child_by_name("src")
            caps = Gst.Caps.from_string("video/x-h265,stream-format=(string)byte-stream")
            appsrc.set_property("caps", caps)

        factory.connect("media-configure", on_configure)

        while True:
            frame = q.tryGet()  # non-blocking
            if not frame:
                continue

            if appsrc:
                data = frame.getData().tobytes()  # <-- numpy → bytes
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                appsrc.emit("push-buffer", buf)

    threading.Thread(target=pump_frames, daemon=True).start()

    # Graceful shutdown on Ctrl-C
    loop = GObject.MainLoop()
    signal.signal(signal.SIGINT, lambda *_: loop.quit())
    loop.run()


if __name__ == "__main__":
    main()
