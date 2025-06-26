import depthai as dai
import cv2

pipeline = dai.Pipeline()
# Create camera node
cam = pipeline.create(dai.node.Camera)
cam.build(dai.CameraBoardSocket.CAM_A)
cam_out = cam.requestOutput(size=(1920, 1080), type=dai.ImgFrame.Type.NV12)
# Create encoder node
video_encoder = pipeline.create(dai.node.VideoEncoder)
video_encoder.setDefaultProfilePreset(30, dai.VideoEncoderProperties.Profile.H265_MAIN)
# Create output link
video_queue = cam_out.createOutputQueue()

# Create links
cam_out.link(video_encoder.input)
video_out = video_encoder.bitstream.createOutputQueue()

# Initialize video writer
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (1920, 1080))

pipeline.start()
try:
    with pipeline:
        while pipeline.isRunning():
            if video_out.has():
                frame = video_out.get().getCvFrame()
                # Write frame to video file
                out.write(frame)
finally:
    # Ensure proper cleanup
    out.release()
    
