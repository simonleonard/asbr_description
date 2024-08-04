# Developer: Seyi R. Afolayan 
# Credits: Adapted from Dennis Stogl 
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declarations
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5", 
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
            default_value="ur_robot_driver",
            description="Package with the controller's configuration in 'config' folder.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
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
            default_value=PathJoinSubstitution([FindPackageShare("asbr_description"), "config", "ur5", "default_kinematics.yaml"]),
            description="The calibration configuration of the actual robot used.", 
            # I might need to run the calibration code recommended on the ur_robot_driver page.
        ),
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="Prefix of the joint names, useful for multi-robot setup",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            # Set to false when using real UR5x
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
            default_value="10",
            description="Timeout used when spawning controllers.",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
            description="Initially loaded robot controller",
        ),
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            # Set to true if not using a rviz file.
            default_value="false",
            description="Launch RViz?",
        ),
        DeclareLaunchArgument(
            "headless_mode",
            # Set to true if using the real UR5x
            default_value="false",
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
            # This is the IP of your device, PC or laptop.
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
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
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

    # Base Launch 
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("ur_robot_driver"), "/launch/ur_control.launch.py"]),
        launch_arguments={
            "ur_type": ur_type, 
            "robot_ip": robot_ip,
            "safety_limits": safety_limits,
            "safety_pos_margin": safety_pos_margin,
            "safety_k_position": safety_k_position,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "kinematics_params_file": kinematics_params_file,
            "tf_prefix": tf_prefix,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "controller_spawner_timeout": controller_spawner_timeout,
            "initial_joint_controller": initial_joint_controller,
            "activate_joint_controller": activate_joint_controller,
            "headless_mode": headless_mode,
            "launch_rviz": launch_rviz,
            "launch_dashboard_client": launch_dashboard_client,
            "use_tool_communication": use_tool_communication,
            "tool_parity": tool_parity,
            "tool_baud_rate": tool_baud_rate,
            "tool_stop_bits": tool_stop_bits,
            "tool_rx_idle_chars": tool_rx_idle_chars, 
            "tool_tx_idle_chars": tool_tx_idle_chars, 
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
            "tool_voltage" : tool_voltage,
            "reverse_ip": reverse_ip,
            "script_command_port": script_command_port,
            "reverse_port": reverse_port,
            "script_sender_port": script_sender_port,
            "trajectory_port": trajectory_port,
        }.items(),
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "ur5e.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        rviz_node
    ]

    return LaunchDescription(declared_arguments + [base_launch] + nodes_to_start)