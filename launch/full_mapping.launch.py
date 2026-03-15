from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # 1. Il Ponte (Bridge) tra Gazebo e ROS 2
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ]
    )

    # 2. Il "Cervello" di RTAB-Map (Odometria Visiva e Mappatura)
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments={
            'rtabmap_args': '--delete_db_on_start',
            'rgb_topic': '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            'depth_topic': '/depth_camera',
            'camera_info_topic': '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info',
            'frame_id': 'camera_link',
            'visual_odometry': 'true',
            'approx_sync': 'true',
            'use_sim_time': 'true',
            'qos': '2',
            'rtabmapviz': 'false', # Lo lanciamo separatamente sotto per avere più controllo
        }.items()
    )

    # 3. L'Interfaccia Grafica (RTAB-Map Viz)
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'approx_sync': True,
            'qos_image': 2,
            'qos_camera_info': 2,
        }],
        remappings=[
            ('odom', '/rtabmap/odom'),
            ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
            ('depth/image', '/depth_camera'),
            ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info'),
        ]
    )

    # Restituiamo tutti i nodi a ROS 2 perché li avvii insieme
    return LaunchDescription([
        bridge_node,
        rtabmap_launch,
        rtabmap_viz_node
    ])
