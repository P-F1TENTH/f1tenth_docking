import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='f1tenth_docking',
            executable='docking_node',
            name='docking_node'),
  ])

