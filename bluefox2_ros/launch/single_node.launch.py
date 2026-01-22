from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
       
    device_name_arg = DeclareLaunchArgument(
        "device",
        default_value="",
        description="Device model name",
    )
    device = LaunchConfiguration("device")

    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Camera namespace",
    )
    namespace = LaunchConfiguration("namespace")    

    camera_calib_url = PythonExpression(["'file://", get_package_share_directory("bluefox2_ros"), "/config/calib_", device, ".yaml'"])
    camera_calib_url_arg = DeclareLaunchArgument(
        "calib_url",
        default_value=camera_calib_url,
        description="URL to the camera calibration file (e.g.: file://calib.yaml)",
    )
    calib_url = LaunchConfiguration("calib_url")    

    config_file_path = PathJoinSubstitution(
        [
            get_package_share_directory('bluefox2_ros'),
            'config',
            PythonExpression(["'camera_", device, ".config.yaml'"])
        ]
    )

    bluefox2_node = Node(
        name=["bluefox2_", device, "_node"],
        namespace=namespace,
        package="bluefox2_ros",
        executable="bluefox2_ros",
        parameters=[config_file_path,
                    {"camera_calibration_url": calib_url}
                    ]
    )
    
    return LaunchDescription([
        device_name_arg,
        namespace_arg,
        camera_calib_url_arg,
        bluefox2_node
    ])