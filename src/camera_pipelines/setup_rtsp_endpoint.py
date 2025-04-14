#!/usr/bin/env python3

import argparse
import subprocess
import socket
import time
import sys
import re
import os
from urllib.parse import urlparse

def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Setup and verify RTSP camera endpoint for webRTC-video')
    parser.add_argument('--rtsp-url', type=str, default='rtsp://192.168.2.250:554/stream',
                      help='RTSP camera URL (default: rtsp://192.168.2.250:554/stream)')
    parser.add_argument('--test-connectivity', action='store_true',
                      help='Test connectivity to the camera')
    parser.add_argument('--check-codec', action='store_true',
                      help='Check codec of the RTSP stream')
    parser.add_argument('--test-gstreamer', action='store_true',
                      help='Test GStreamer pipeline with the camera')
    parser.add_argument('--allow-firewall', action='store_true',
                      help='Configure firewall to allow RTSP connections')
    
    return parser.parse_args()

def check_dependencies():
    """Check if necessary dependencies are installed"""
    print("Checking required dependencies...")
    
    dependencies = {
        'gst-launch-1.0': 'gstreamer1.0-tools',
        'ffprobe': 'ffmpeg'
    }
    
    missing_deps = []
    
    for cmd, package in dependencies.items():
        try:
            subprocess.run(['which', cmd], check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            print(f"✓ {cmd} is installed")
        except subprocess.CalledProcessError:
            print(f"✗ {cmd} is not installed (package: {package})")
            missing_deps.append(package)
    
    if missing_deps:
        print("\nMissing dependencies. Install them with:")
        print(f"sudo apt update && sudo apt install -y {' '.join(missing_deps)}")
        return False
    
    return True

def test_connectivity(rtsp_url):
    """Test network connectivity to the RTSP server"""
    print(f"\nTesting connectivity to RTSP server: {rtsp_url}")
    
    # Parse URL to get host and port
    parsed_url = urlparse(rtsp_url)
    host = parsed_url.hostname
    port = parsed_url.port or 554  # Default RTSP port is 554
    
    print(f"Testing connectivity to {host}:{port}...")
    
    # Check if host is reachable
    try:
        # Try to ping the host first
        ping_result = subprocess.run(['ping', '-c', '1', '-W', '2', host], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE)
        
        if ping_result.returncode == 0:
            print(f"✓ Host {host} is reachable (ping successful)")
        else:
            print(f"⚠ Host {host} is not responding to ping (not necessarily an error)")
    except Exception as e:
        print(f"⚠ Error while pinging {host}: {str(e)}")
    
    # Check if RTSP port is open
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(2)
        result = sock.connect_ex((host, port))
        if result == 0:
            print(f"✓ Port {port} is open on {host}")
        else:
            print(f"✗ Port {port} is closed on {host}")
            print("  This could indicate the camera is not accessible.")
            return False
        sock.close()
    except Exception as e:
        print(f"✗ Error connecting to {host}:{port}: {str(e)}")
        return False
    
    return True

def check_codec(rtsp_url):
    """Check the codec of the RTSP stream using ffprobe"""
    print(f"\nChecking codec of RTSP stream: {rtsp_url}")
    
    try:
        result = subprocess.run(
            ['ffprobe', '-v', 'error', '-select_streams', 'v:0', 
             '-show_entries', 'stream=codec_name', '-of', 'default=noprint_wrappers=1:nokey=1', 
             rtsp_url],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=10
        )
        
        if result.returncode == 0 and result.stdout.strip():
            codec = result.stdout.strip()
            print(f"✓ Stream codec: {codec}")
            
            if codec.lower() == 'h264':
                print("✓ Camera is using H.264 codec, which is ideal for direct RTSP usage")
                print("  You can use type=\"rtsp\" in the webRTC-video component")
            elif codec.lower() in ['h265', 'hevc']:
                print("ℹ Camera is using H.265/HEVC codec")
                print("  Use type=\"rtsp-transcode\" in the webRTC-video component")
            else:
                print(f"⚠ Camera is using {codec} codec")
                print("  Use type=\"rtsp-transcode\" in the webRTC-video component")
            
            return codec
        else:
            print(f"✗ Failed to detect codec: {result.stderr}")
            return None
    except subprocess.TimeoutExpired:
        print("✗ Timeout while detecting codec - camera may be unreachable")
        return None
    except Exception as e:
        print(f"✗ Error detecting codec: {str(e)}")
        return None

def test_gstreamer_pipeline(rtsp_url, codec=None):
    """Test GStreamer pipeline with the camera"""
    print(f"\nTesting GStreamer pipeline with: {rtsp_url}")
    
    if not codec:
        # Try to detect the codec if not provided
        try:
            result = subprocess.run(
                ['ffprobe', '-v', 'error', '-select_streams', 'v:0', 
                '-show_entries', 'stream=codec_name', '-of', 'default=noprint_wrappers=1:nokey=1', 
                rtsp_url],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=5
            )
            if result.returncode == 0 and result.stdout.strip():
                codec = result.stdout.strip().lower()
                print(f"Detected codec: {codec}")
        except:
            print("Could not auto-detect codec, assuming h265")
            codec = 'h265'
    
    # Choose the right elements based on codec
    if codec and codec.lower() in ['h264', 'avc']:
        depay = 'rtph264depay'
        parse = 'h264parse'
        decode = 'avdec_h264'
    else:
        # Default to H.265 elements
        depay = 'rtph265depay'
        parse = 'h265parse'
        decode = 'avdec_h265'
    
    # Build the pipeline
    pipeline_cmd = (
        f"gst-launch-1.0 -v rtspsrc location={rtsp_url} latency=0 "
        f"! {depay} ! {parse} ! {decode} ! videoconvert ! autovideosink"
    )
    
    print(f"Testing pipeline: {pipeline_cmd}")
    print("This will open a window showing the camera stream if successful.")
    print("Press Ctrl+C to end the test.")
    
    try:
        # Run the pipeline for a few seconds
        process = subprocess.Popen(
            pipeline_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        
        # Let it run for up to 10 seconds
        time.sleep(2)  # Give a moment for the window to appear
        print("Pipeline running. If you see a video window, the test is successful.")
        print("Waiting up to 8 more seconds...")
        
        # Wait for up to 8 more seconds or until the user interrupts
        start_time = time.time()
        while time.time() - start_time < 8:
            if process.poll() is not None:
                # Process ended on its own
                break
            time.sleep(0.5)
        
        # Check if it's still running
        if process.poll() is None:
            print("✓ GStreamer pipeline test successful!")
            process.terminate()
            return True
        else:
            # Get the output to see what went wrong
            stdout, stderr = process.communicate()
            print(f"✗ GStreamer pipeline failed: {stderr.decode()}")
            return False
    
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
        if 'process' in locals():
            process.terminate()
        return True
    except Exception as e:
        print(f"✗ Error testing GStreamer pipeline: {str(e)}")
        return False

def configure_firewall(rtsp_url):
    """Configure firewall to allow RTSP connections"""
    print("\nConfiguring firewall for RTSP connections...")
    
    # Parse URL to get host and port
    parsed_url = urlparse(rtsp_url)
    host = parsed_url.hostname
    port = parsed_url.port or 554  # Default RTSP port is 554
    
    try:
        # Check if ufw is installed and active
        ufw_status = subprocess.run(['ufw', 'status'], 
                                  stdout=subprocess.PIPE, 
                                  stderr=subprocess.PIPE,
                                  text=True)
        
        if ufw_status.returncode == 0 and "Status: active" in ufw_status.stdout:
            print("UFW firewall is active")
            
            # Allow RTSP port
            subprocess.run(['sudo', 'ufw', 'allow', str(port) + '/tcp'], 
                          stdout=subprocess.PIPE, 
                          stderr=subprocess.PIPE)
            print(f"✓ Added rule to allow TCP port {port}")
            
            # Allow RTSP over UDP too
            subprocess.run(['sudo', 'ufw', 'allow', str(port) + '/udp'], 
                          stdout=subprocess.PIPE, 
                          stderr=subprocess.PIPE)
            print(f"✓ Added rule to allow UDP port {port}")
            
            return True
        else:
            print("UFW firewall is not active, no configuration needed")
            return True
    
    except Exception as e:
        print(f"⚠ Error configuring firewall: {str(e)}")
        return False

def generate_html_component(rtsp_url):
    """Generate HTML component for webRTC video"""
    print("\nGenerating HTML component code for webRTC-video:")
    
    html = f"""  <webrtc-video-device 
    id="camera1" 
    host="transitiverobotics.com" 
    ssl="true"
    jwt="{{jwt}}"
    count="1"
    timeout="1800"
    type="rtsp-transcode"
    source="{rtsp_url}"
  />"""
    
    print("\n" + html + "\n")
    print("Copy this HTML code to your application to use the camera with webRTC-video.")
    print("Replace {{jwt}} with your actual JWT token.")

def main():
    args = parse_arguments()
    
    print("===================================")
    print("RTSP Camera Setup for webRTC-video")
    print("===================================")
    
    # Check dependencies first
    if not check_dependencies():
        sys.exit(1)
    
    # Test connectivity if requested
    if args.test_connectivity:
        if not test_connectivity(args.rtsp_url):
            print("\n⚠ Warning: Connectivity issues detected. Setup may not work properly.")
    
    # Check codec if requested
    codec = None
    if args.check_codec:
        codec = check_codec(args.rtsp_url)
    
    # Configure firewall if requested
    if args.allow_firewall:
        configure_firewall(args.rtsp_url)
    
    # Test GStreamer pipeline if requested
    if args.test_gstreamer:
        test_gstreamer_pipeline(args.rtsp_url, codec)
    
    # Generate HTML component
    generate_html_component(args.rtsp_url)
    
    print("\nSetup Complete!")
    print("To use the camera with the webRTC-video component:")
    print("1. Make sure the camera is accessible from your application")
    print("2. Use the generated HTML component with type=\"rtsp-transcode\"")
    print("3. Provide a valid JWT token for authentication")
    
    if not (args.test_connectivity or args.check_codec or args.test_gstreamer):
        print("\nTip: Run this script with --test-connectivity --check-codec --test-gstreamer")
        print("to perform a full validation of your camera setup.")

if __name__ == "__main__":
    main() 