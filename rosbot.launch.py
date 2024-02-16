from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Retrieve the healthcheck and namespace arguments
    healthcheck = LaunchConfiguration("healthcheck")
    namespace = LaunchConfiguration("namespace")

    # Define the healthcheck node
    healthcheck_node = Node(
        package="healthcheck_pkg",
        executable="healthcheck_node",
        name="healthcheck_rosbot",
        namespace=namespace,
        output="screen",
        condition=IfCondition(healthcheck)
    )

    # Include 'combined.launch.py' with forwarded arguments
    combined_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('rosbot_xl_bringup'), '/launch/combined.launch.py'
        ]),
        launch_arguments={arg: LaunchConfiguration(arg) for arg in context.launch_configurations}.items()
    )

    return [combined_launch, healthcheck_node]

def generate_launch_description():
    return LaunchDescription([
        # Declare the 'healthcheck' argument
        DeclareLaunchArgument(
            'healthcheck',
            default_value='False',
            description='Enable health check for ROSbot XL node.'
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
            description="Namespace for nodes",
        ),
        # Setup for including 'combined.launch.py' and adding the healthcheck node
        OpaqueFunction(function=launch_setup),
    ])
