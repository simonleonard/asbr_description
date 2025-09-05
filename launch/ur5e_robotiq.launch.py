# Developer: Seyi R. Afolayan 
# Credits: Adapted from Dennis Stogl 
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    # Declarations
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e", 
            description="Type/Series of used Robot",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="172.22.22.2",
            description="IP address of the UR robot",
        ), 
        DeclareLaunchArgument(
            "safety_limits", 
            default_value="true",
            description="Enable the safety limits controller if true",
        ), 
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="asbr_description",
            description="Package with the controller's configuration in 'config' folder. The package can be ur_robot_driver but I copied it because I want to include the gripper",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml",
            description="YAML file with the conroller's configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="asbr_description",
            description="Description package with custom/default robot URDF/XACRO files.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="asbr.urdf.xacro",
            description="The URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "kinematics_params_file",
            default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", "ur5", "default_kinematics.yaml"]),
            description="The calibration configuration of the actual robot used.", 
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true", 
            description="Start the robot with fake hardware.",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands", 
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations.",
        ),
        DeclareLaunchArgument(
            "controller_spawner_timeout", 
            default_value="60",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            default_value="true",
            description="Enable headless mode for real robot control.",
        ),
        DeclareLaunchArgument(
            "launch_dashboard_client",
            default_value="false",
            description="Launch Dashboard Client?",
        ),
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        ),
        DeclareLaunchArgument(
            "tool_parity",
            default_value="0",
            description="Parity communication for serial communication.",
        ),
        DeclareLaunchArgument(
            "tool_baud_rate",
            default_value="115200",
            description="Baud rate configuration for serial communication",
        ),
        DeclareLaunchArgument(
            "tool_stop_bits",
            default_value="1",
            description="Stop bits configuration for serial communication.",
        ),
        DeclareLaunchArgument(
            "tool_rx_idle_chars", 
            default_value="1.5",
            description="RX (Reciever) idle chars configuration for serial communication.",
        ),
        DeclareLaunchArgument(
            "tool_tx_idle_chars",
            default_value="3.5",
            description="TX (Transmitter) idle chars configuration for serial communication.",
        ),
        DeclareLaunchArgument(
            "tool_device_name",
            default_value="/tmp/ttyUR",
            description="File descriptor for the tool communication device.",
        ),
        DeclareLaunchArgument(
            "tool_tcp_port",
            default_value="54321",
            description="Remote port for the tool's serial device.",
        ),
        DeclareLaunchArgument(
            "tool_voltage",
            default_value="0",
            description="Tool voltage that will be setup.",
        ),
        DeclareLaunchArgument(
            "reverse_ip",
            default_value="172.22.22.10",
            description="IP for the robot controller to communicate back to the driver.",
        ),
        DeclareLaunchArgument(
            "script_command_port",
            default_value="50004",
            description="Port for forwarding URScript commands to the robot.",
        ),
        DeclareLaunchArgument(
            "reverse_port",
            default_value="50001",
            description="Port for sending cyclic instructions from the driver to the robot controller.",
        ),
        DeclareLaunchArgument(
            "script_sender_port",
            default_value="50002",
            description="Interface to query external_control URScript on this port.",
        ),
        DeclareLaunchArgument(
            "trajectory_port", 
            default_value="50003",
            description="Port for trajectory control", 
        ),
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="false",
            description="Simulate with Ignition Gazebo",
        ),
        DeclareLaunchArgument(
            "sim_isaac",
            default_value="false",
            description="Simulate with Isaac Sim",
        ),
        DeclareLaunchArgument(
            "com_port",
            default_value="/dev/ttyUSB0",
            description="Communication port for the gripper",
        ),
    ]

    # Initializations
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    kinematics_params_file = LaunchConfiguration("kinematics_params_file")
    tf_prefix = LaunchConfiguration("tf_prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_parity = LaunchConfiguration("tool_parity")
    tool_baud_rate = LaunchConfiguration("tool_baud_rate")
    tool_stop_bits = LaunchConfiguration("tool_stop_bits")
    tool_rx_idle_chars = LaunchConfiguration("tool_rx_idle_chars")
    tool_tx_idle_chars = LaunchConfiguration("tool_tx_idle_chars")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    tool_voltage = LaunchConfiguration("tool_voltage")
    reverse_ip = LaunchConfiguration("reverse_ip")
    script_command_port = LaunchConfiguration("script_command_port")
    reverse_port = LaunchConfiguration("reverse_port")
    script_sender_port = LaunchConfiguration("script_sender_port")
    trajectory_port = LaunchConfiguration("trajectory_port")
    sim_ignition = LaunchConfiguration("sim_ignition")
    sim_isaac = LaunchConfiguration("sim_isaac")
    com_port = LaunchConfiguration("com_port")

    # Combine the robot and gripper description commands into one
    combined_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=", safety_limits,
            " ",
            "safety_pos_margin:=", safety_pos_margin,
            " ",
            "safety_k_position:=", safety_k_position,
            " ",
            "name:=", "ur",
            " ",
            "ur_type:=", ur_type,
            " ",
            "tf_prefix:=", tf_prefix,
            " ",
            "sim_ignition:=", sim_ignition,
            " ",
            "sim_isaac:=", sim_isaac,
            " ",
            "use_fake_hardware:=", use_fake_hardware,
            " ",
            "fake_sensor_commands:=", fake_sensor_commands,
            " ",
            "prefix:=", "",
            " ",
            "include_ros2_control:=", "true",
            " ",
            "com_port:=", com_port,
        ]
    )

    # Create the combined robot description
    robot_description = {"robot_description": combined_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "ur5e.rviz"]
    )

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", "controllers.yaml"]
    )

    # Boiler plate
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    # Controllers
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                controller_spawner_timeout,
            ]
            + inactive_flags
            + controllers,
        )

    controllers_active = [
        "ur_joint_state_broadcaster",
        "robotiq_joint_state_broadcaster",
        "ur5e_controller",
        "robotiq_controller"
    ]
    
    controllers_inactive = [
        "io_and_status_controller",
        "speed_scaling_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "scaled_joint_trajectory_controller",
        "forward_velocity_controller",
        "forward_position_controller",
        #"robotiq_activation_controller"
    ]

    controller_spawners = [
        controller_spawner(controllers_active)] + [
        controller_spawner(controllers_inactive, active=False)
    ]

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
    )

    nodes_to_start = [
        robot_state_publisher,
        rviz,
        controller_manager
    ] + controller_spawners

    return LaunchDescription(declared_arguments +  nodes_to_start)
