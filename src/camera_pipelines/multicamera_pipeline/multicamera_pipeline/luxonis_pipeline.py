# REQUIRENMENTS rtsp-simple-server
# sudo apt-get install rtsp-simple-server

import depthai as dai
import time
import cv2
import threading
import queue
import numpy as np
import subprocess
import os
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

def getPipeline(stereo=False):
    # Start defining a pipeline
    pipeline = dai.Pipeline()

    # Define a source - color camera
    cam_rgb = pipeline.create(dai.node.ColorCamera)
    
    # Configure RGB camera
    cam_rgb.setPreviewSize(640, 360)  # Set preview size
    cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam_rgb.setInterleaved(False)
    cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    cam_rgb.setFps(30)  # Set frame rate

    # Create output
    xout_rgb = pipeline.create(dai.node.XLinkOut)
    xout_rgb.setStreamName("rgb")
    cam_rgb.preview.link(xout_rgb.input)
    
    return pipeline

class RTSPStreamer:
    def __init__(self, port):
        self.port = port
        self.frame_queue = queue.Queue(maxsize=10)
        self.is_running = False
        self.stream_thread = None
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0
        self.pipeline = None
        self.appsrc = None
        self.rtsp_server = None
        
    def start(self):
        self.is_running = True
        self.stream_thread = threading.Thread(target=self._stream_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
    def stop(self):
        self.is_running = False
        if self.stream_thread:
            self.stream_thread.join()
            
    def update_frame(self, frame):
        if not self.frame_queue.full():
            self.frame_queue.put(frame)
            self.frame_count += 1
            
            # Calculate FPS every second
            current_time = time.time()
            if current_time - self.last_fps_time >= 1.0:
                self.fps = self.frame_count
                self.frame_count = 0
                self.last_fps_time = current_time
                print(f"RTSP Stream FPS: {self.fps}")
            
    def _stream_loop(self):
        # Initialize GStreamer
        Gst.init(None)
        
        # Start RTSP server
        rtsp_config = f"""
        paths:
            stream{self.port}:
                source: udp://127.0.0.1:{self.port}
                sourceProtocol: udp
        """
        
        config_path = f"/tmp/rtsp-config-{self.port}.yml"
        with open(config_path, 'w') as f:
            f.write(rtsp_config)
            
        self.rtsp_server = subprocess.Popen(
            ['rtsp-simple-server', config_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Create pipeline
        pipeline_str = (
            f"appsrc name=mysrc ! videoconvert ! x264enc tune=zerolatency bitrate=500 "
            f"speed-preset=superfast ! rtph264pay ! udpsink host=127.0.0.1 port={self.port}"
        )
        
        print(f"Starting RTSP stream with pipeline: {pipeline_str}")
        
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsrc = self.pipeline.get_by_name("mysrc")
        
        # Set appsrc properties
        self.appsrc.set_property("format", Gst.Format.TIME)
        self.appsrc.set_property("is-live", True)
        self.appsrc.set_property("do-timestamp", True)
        self.appsrc.set_property("stream-type", 0)  # GST_APP_STREAM_TYPE_STREAM
        
        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("Failed to start pipeline")
            return
            
        print(f"RTSP stream started on port {self.port}")
        
        while self.is_running:
            try:
                frame = self.frame_queue.get(timeout=1.0)
                
                # Convert frame to GStreamer buffer
                height, width = frame.shape[:2]
                buffer = Gst.Buffer.new_wrapped(frame.tobytes())
                
                # Push buffer to appsrc
                ret = self.appsrc.emit("push-buffer", buffer)
                if ret != Gst.FlowReturn.OK:
                    print(f"Failed to push buffer to pipeline: {ret}")
                    break
                    
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Error in RTSP stream: {str(e)}")
                break
                
        # Cleanup
        self.pipeline.set_state(Gst.State.NULL)
        if self.rtsp_server:
            self.rtsp_server.terminate()
            os.remove(config_path)
        print(f"RTSP stream on port {self.port} stopped")

def test_rtsp_connection(host, port):
    """Test if RTSP stream is accessible"""
    try:
        # Try to connect to the RTSP stream
        cap = cv2.VideoCapture(f"rtsp://{host}:8554/stream{port}", cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'H264'))
        
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret and frame is not None:
                print(f"Successfully connected to RTSP stream at {host}:8554/stream{port}")
                return True
            else:
                print(f"Failed to read frame from RTSP stream at {host}:8554/stream{port}")
        else:
            print(f"Failed to open RTSP stream at {host}:8554/stream{port}")
    except Exception as e:
        print(f"Error testing RTSP connection: {str(e)}")
    return False

def main():
    try:
        # Get all available devices
        device_infos = dai.Device.getAllAvailableDevices()
        print(f'Found {len(device_infos)} devices')

        # Dictionary to store device queues and RTSP streamers
        device_queues = {}
        devices = {}
        rtsp_streamers = {}
        base_port = 8554  # Starting port for RTSP streams
        stream_host = "127.0.0.1"  # IP address for streaming

        # Start pipeline for each device
        for idx, device_info in enumerate(device_infos):
            try:
                # Create device
                device = dai.Device(device_info)
                
                # Get the MXID for this device
                mxid = device.getMxId()
                print(f"Initializing device with MXID: {mxid}")
                
                # Start pipeline for this device
                device.startPipeline(getPipeline())
                
                # Store the device and its output queue
                devices[mxid] = device
                device_queues[f"rgb-{mxid}"] = device.getOutputQueue(name="rgb")
                
                # Create and start RTSP streamer for this device
                port = base_port + idx
                rtsp_streamer = RTSPStreamer(port)
                rtsp_streamer.start()
                rtsp_streamers[mxid] = rtsp_streamer
                
                print(f"Successfully initialized device {mxid} with RTSP stream on port {port}")
                
                # Test RTSP connection
                time.sleep(2)  # Wait for stream to start
                test_rtsp_connection(stream_host, port)
                
            except Exception as e:
                print(f"Error initializing device {device_info.getMxId()}: {str(e)}")
                continue

        # Main loop to process frames from all cameras
        while True:
            for mxid, device in devices.items():
                try:
                    # Get RGB output from the current device
                    rgb_queue = device_queues[f"rgb-{mxid}"]
                    rgb_data = rgb_queue.get()
                    
                    # Get the frame data
                    frame = rgb_data.getCvFrame()
                    
                    # Update RTSP stream
                    rtsp_streamers[mxid].update_frame(frame)
                    
                except Exception as e:
                    print(f"Error processing frame from device {mxid}: {str(e)}")
                    continue
                    
            time.sleep(0.01)  # Small delay to prevent CPU overload
            
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error in main loop: {str(e)}")
    finally:
        # Stop all RTSP streamers
        for streamer in rtsp_streamers.values():
            streamer.stop()
            
        # Clean up devices
        for device in devices.values():
            device.close()

if __name__ == "__main__":
    main()




