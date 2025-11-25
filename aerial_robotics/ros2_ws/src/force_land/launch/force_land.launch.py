from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Lancia il nodo force_land."""
    
    # Define il nodo che verrà lanciato
    force_land_node = Node(
        # Il nome del tuo pacchetto ROS 2
        package='force_land',
        
        # Il nome dell'eseguibile (definito in CMakeLists.txt, nel tuo caso 'force_land')
        executable='force_land',
        
        # Nome del nodo, come apparirà in ros2 node list
        name='force_land_monitor',
        
        # Tipo di output della console
        output='screen',
        
    )

    return LaunchDescription([
        force_land_node,
    ])
