import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Trova la cartella del pacchetto ufficiale di Nav2
    nav2_launch_dir = os.path.join(FindPackageShare('nav2_bringup').find('nav2_bringup'), 'launch')
    
    # Percorso esatto al file di configurazione
    params_file = os.path.expanduser('~/autonomous-drone-navigation/config/nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',          # Sincronizzazione con Gazebo
                'params_file': params_file       # Il file con /rtabmap/map
            }.items(),
        )
    ])
