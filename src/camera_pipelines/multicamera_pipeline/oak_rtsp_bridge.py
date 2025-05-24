#!/usr/bin/env python3
"""
Bridges depthai_ros_driver H-265 packets to an RTSP mount point
using GStreamer’s GstRtspServer. One node == one camera stream.
"""
import queue, threading, rclpy, gi
from depthai_ros_msgs.msg import H265Packet
from rclpy.node import Node

gi.require_version("Gst", "1.0")
gi.require_version("GstRtspServer", "1.0")
from gi.repository import Gst, GstRtspServer, GObject

Gst.init(None)


class RTSPBridge(Node):
    def __init__(self):
        super().__init__("oak_rtsp_bridge")

        self.declare_parameter("topic", "/front_camera/video/h265")
        self.declare_parameter("port", 8554)
        self.declare_parameter("mount", "front_camera")

        topic = self.get_parameter("topic").value
        port = self.get_parameter("port").value
        mount = self.get_parameter("mount").value

        self.q = queue.Queue(maxsize=300)
        self.create_subscription(H265Packet, topic, self._cb, 10)

        # ---- RTSP server ---------------------------------------------------
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

        def push_data(src, _):
            try:
                buf = self.q.get_nowait()
            except queue.Empty:
                return Gst.FlowReturn.OK
            gst_buf = Gst.Buffer.new_allocate(None, len(buf), None)
            gst_buf.fill(0, buf)
            return src.emit("push-buffer", gst_buf)

        def media_configure(_, media):
            appsrc = media.get_element().get_child_by_name("src")
            caps = Gst.Caps.from_string(
                "video/x-h265,stream-format=(string)byte-stream"
            )
            appsrc.set_property("caps", caps)
            appsrc.connect("need-data", push_data)

        factory.connect("media-configure", media_configure)

        threading.Thread(
            target=GObject.MainLoop().run, daemon=True
        ).start()

        self.get_logger().info(
            f"RTSP ready at rtsp://<host>:{port}/{mount}"
        )

    # ROS → Queue
    def _cb(self, msg: H265Packet):
        try:
            self.q.put(msg.data, block=False)
        except queue.Full:
            self.get_logger().warn("RTSP queue full – dropping frame")


def main(args=None):
    rclpy.init(args=args)
    node = RTSPBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
