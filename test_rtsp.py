#!/usr/bin/env python3
import cv2
import time
import os

# Set GStreamer debug level
os.environ["GST_DEBUG"] = "3,rtspsrc:5,rtph265depay:5,h265parse:5,avdec_h265:5"

# Test different pipelines
pipelines = [
    # Simple pipeline with authentication
    "rtspsrc location=rtsp://admin:admin@192.168.2.250:554/stream protocols=tcp ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false",
    
    # Alternative with properties separated
    "rtspsrc location=rtsp://admin:admin@192.168.2.250:554/stream protocols=tcp latency=0 ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false"
]

for i, pipeline in enumerate(pipelines):
    print(f"\nTrying pipeline {i+1}: {pipeline}")
    
    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        print(f"Failed to open camera with pipeline {i+1}")
        continue
        
    # Try to read a frame
    ret, frame = cap.read()
    if not ret:
        print(f"Failed to read frame with pipeline {i+1}")
        cap.release()
        continue
        
    print(f"Success with pipeline {i+1}! Frame shape: {frame.shape}")
    
    # Read a few more frames
    for j in range(5):
        ret, frame = cap.read()
        if ret:
            print(f"  Frame {j+1} received, size: {frame.shape}")
        else:
            print(f"  Frame {j+1} failed")
        time.sleep(0.1)
    
    cap.release()
    print(f"Pipeline {i+1} test complete")

print("\nAll tests complete") 