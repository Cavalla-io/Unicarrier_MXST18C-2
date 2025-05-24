from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def oak_nodes(name: str, ip_or_mxid: str | None = None):
    """
    Return a Camera component *plus* its RTSP bridge node for one OAK.
    """
    params = {
        "i_pipeline_type": "video",            # disable depth/rect/IMU
        "video": {
            "resolution": "1080",
            "fps": 30,
            "encoder_profile": "h265_main",
        },
        "i_poe_ip": ip_or_mxid if ip_or_mxid and "." in ip_or_mxid else "",
        "i_mxid":   ip_or_mxid if ip_or_mxid and "." not in ip_or_mxid else "",
        "enable_depth": False,
        "rectify_rgb": False,
        "rsp_use_composition": False,
        "publish_tf_from_calibration": False,
        "imu_from_descr": False,
    }

    cam = ComposableNode(
        package="depthai_ros_driver",
        plugin="depthai_ros_driver::Camera",
        name=name,
        namespace=name,
        parameters=[params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    rtsp = Node(
        package="multicamera_pipeline",
        executable="oak_rtsp_bridge",
        name=f"{name}_rtsp",
        parameters=[{
            "topic": f"/{name}/video/h265",
            "mount": name,          # rtsp://host:8554/{name}
            "port": 8554,
        }],
        output="screen",
    )
    return [cam, rtsp]


def generate_launch_description() -> LaunchDescription:
    # Friendly name : PoE IP  (or MXID for USB)
    cameras = {
        "front_camera": "192.168.50.10",
        "fork_camera":  "192.168.50.11",
    }

    container = ComposableNodeContainer(
        name="oak_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            node
            for name, ip in cameras.items()
            for node in oak_nodes(name, ip)
        ],
        output="screen",
    )

    return LaunchDescription([container])
