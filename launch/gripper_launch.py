# Developer: Seyi R. Afolayan 
# Credits: Dennis Stogl

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

# Declaration need for the 2f_85 gripper
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "com_port", 
            default_value="/dev/ttyUSB0", 
            description="Communication port for the gripper",
        ), 
        DeclareLaunchArgument(
            "description_package", 
            default_value="robotiq_description",
            description="Package containing the necessary XACRO/URDFs",
        ),
        DeclareLaunchArgument(
            "description_file", 
            default_value="robotiq_2f_85_gripper.urdf.xacro",
            description="The main XACRO/URDF.",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware", 
            default_value="true",
            description="Start the robot with fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable if using simulation.",
        ),
        DeclareLaunchArgument(
            "include_ros2_control",
            default_value="true",
            description="Include ROS2 control instance.", 
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for joint names",
        ),
    ]

    # Initialize Launch Configurations 
    com_port = LaunchConfiguration("com_port")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commmands = LaunchConfiguration("fake_sensor_commands")
    include_ros2_control = LaunchConfiguration("include_ros2_control")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "prefix:=", prefix, 
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ", 
            "fake_sensor_commands:=", fake_sensor_commmands,
            " ",
            "include_ros2_control:=", include_ros2_control,
            " ",
            "com_port:=", com_port    
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    gripper_yaml_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", "robotiq_controllers.yaml"]
    )

    # Define Nodes 
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, gripper_yaml_file],
        output="both",
    )

    robotiq_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robotiq_activation_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_activation_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # RViz file and node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "view_urdf.rviz"]
    )

    rviz_node = Node(
        package="rviz2", 
        executable="rviz2",
        name="rviz2",
        output="log", 
        arguments=["-d", rviz_config_file]
    )

    # Nodes to start
    nodes_to_start = [
        rviz_node,
        robot_state_publisher_node,
        controller_manager_node, 
        robotiq_gripper_controller_spawner,
        robotiq_activation_controller_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)