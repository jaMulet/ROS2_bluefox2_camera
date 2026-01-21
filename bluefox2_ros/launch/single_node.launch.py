from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
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
   
    camera_calib_path = PathJoinSubstitution(
        [
            get_package_share_directory("bluefox2_ros"),
            "config",
            PythonExpression(["'calib_", device, "' + '.yaml'"])
        ]
    )

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
                    {"camera_calibration_file": camera_calib_path}
                    ]
    )
    
    return LaunchDescription([
        device_name_arg,
        namespace_arg,
        bluefox2_node
    ])