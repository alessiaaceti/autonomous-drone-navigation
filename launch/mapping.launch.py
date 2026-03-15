from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Nodo principale di RTAB-Map (Il "cervello" della mappa)
        Node(
            package='rtabmap_slam', 
            executable='rtabmap', 
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'use_sim_time': True,
                'qos_image': 2,
                'qos_camera_info': 2
            }],
            remappings=[
                ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('depth/image', '/depth_camera'),
                ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info')
            ],
            arguments=['--delete_db_on_start'] # Cancella la vecchia mappa a ogni avvio
        ),
        
        # 2. Nodo per l'interfaccia grafica (rtabmap_viz)
        Node(
            package='rtabmap_viz', 
            executable='rtabmap_viz', 
            output='screen',
            parameters=[{
                'frame_id': 'camera_link',
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'approx_sync': True,
                'use_sim_time': True,
                'qos_image': 2,
                'qos_camera_info': 2
            }],
            remappings=[
                ('rgb/image', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'),
                ('depth/image', '/depth_camera'),
                ('rgb/camera_info', '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info')
            ]
        )
    ])
