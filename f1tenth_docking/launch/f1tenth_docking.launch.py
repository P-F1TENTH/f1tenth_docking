import os

from ament_index_python.packages import get_package_share_directory

import launch
import launch_ros.actions

package_name = "f1tenth_docking"

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'config.yaml'
    )

    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package=package_name,
            executable='docking_action_server',
            name='docking_action_server',
            emulate_tty=True,
            parameters=[config]
        )
  ])


