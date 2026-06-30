import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    desc_pkg = get_package_share_directory('lite6_description')
    hw_pkg = get_package_share_directory('lite6_hardware')
    moveit_pkg = get_package_share_directory('lite6_moveit_config')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'lite6.urdf')
    with open(urdf_file, 'r') as infp: 
        robot_desc = infp.read()
    
    controller_config = os.path.join(hw_pkg, 'config', 'ros2_controllers.yaml')

    # Shadow robot URDF generation: We will create a modified version of the original URDF for the shadow robot. 
    # This involves renaming the links to avoid conflicts with the main robot's links. 
    # The shadow robot will have its own set of links prefixed with "shadow_".
    
    shadow_desc = robot_desc
    links_to_rename = ['link_base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6', 'link_eef']
    for l in links_to_rename:
        shadow_desc = shadow_desc.replace(f'name="{l}"', f'name="shadow_{l}"')
        shadow_desc = shadow_desc.replace(f'link="{l}"', f'link="shadow_{l}"')
        # Replace URDF "White" (1.0 1.0 1.0 1.0) with Cyan (0.0 0.8 1.0 1.0)
        shadow_desc = shadow_desc.replace(
            '<color rgba="1.0 1.0 1.0 1.0"/>', 
            '<color rgba="0.0 0.8 1.0 1.0"/>'
        )
        
        # Replace URDF "Silver" (0.753 0.753 0.753 1.0) with a darker Blue (0.0 0.4 0.8 1.0)
        shadow_desc = shadow_desc.replace(
            '<color rgba="0.753 0.753 0.753 1.0"/>', 
            '<color rgba="0.0 0.4 0.8 1.0"/>'
        )

    # 1. Start the main ros2_control_node and main robot_state_publisher
    control_node = Node(
        package="controller_manager", 
        executable="ros2_control_node", 
        parameters=[{'robot_description': robot_desc}, controller_config], 
        output="screen"
    )
    
    rsp_node = Node(
        package='robot_state_publisher', 
        executable='robot_state_publisher', 
        parameters=[{'robot_description': robot_desc}, {'publish_frequency': 200.0}]
    )

    # 2. Spawners
    jsb_spawner = Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"])
    ctc_spawner = Node(package="controller_manager", executable="spawner", arguments=["ctc_controller"])
    jtc_spawner = Node(package="controller_manager", executable="spawner", arguments=["lite6_arm_controller"])

    delay_jtc_after_ctc = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=ctc_spawner, on_exit=[jtc_spawner])
    )

    # 3. MoveIt2
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'allow_trajectory_execution': 'true',
            'moveit_manage_controllers': 'true',
            'pipeline': 'pilz_industrial_motion_planner',
        }.items()
    )

    # 4. RViz
    rviz_config_file = os.path.join(desc_pkg, 'rviz', 'clean.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    else:
        rviz_node = Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', rviz_config_file])

    # 5. Shadow Tracker (Python Script)
    shadow_tracker_node = Node(package='lite6_hmi', executable='shadow_tracker', output='screen')

    # 6. Shadow RSP (using isolated topic and modified URDF)
    shadow_rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='shadow_robot_state_publisher',
        parameters=[{
            'robot_description': shadow_desc,
            'publish_frequency': 200.0
        }],
        remappings=[
            ('/joint_states', '/shadow/joint_states'),
            ('/robot_description', '/shadow_robot_description')
        ]
    )

    # 7. Static TF to anchor the shadow robot
    shadow_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'link_base', 'shadow_link_base'],
        output='screen'
    )

    # 8. HMI
    hmi_node = TimerAction(period=4.0, actions=[Node(package='lite6_hmi', executable='gui_main', output='screen')])

    return LaunchDescription([
        control_node, rsp_node, jsb_spawner, ctc_spawner, delay_jtc_after_ctc,
        move_group_launch, rviz_node,
        shadow_tracker_node, shadow_rsp_node, shadow_tf_node,
        hmi_node
    ])