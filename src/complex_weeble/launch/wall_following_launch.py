import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        #Main function that runs the angle based control node
        launch_ros.actions.Node(
            package='complex_weeble',
            executable='wall_following',
            name='wall_following'),
        
        #Node that runs controller
        launch_ros.actions.Node(
            package='sensor_data',
            executable='get_ultra_sonic',
            name='get_ultra_sonic'),

    ])
