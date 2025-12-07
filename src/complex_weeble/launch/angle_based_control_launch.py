import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        #Main function that runs the angle based control node
        launch_ros.actions.Node(
            package='complex_weeble',
            executable='angle_based_control',
            name='angle_based_control'),
        #Node that runs controller
        launch_ros.actions.Node(
            package='controller',
            executable='dpad_control',
            name='dpad_control'),
        #Node that publishes encoder data
        launch_ros.actions.Node(
            package='sensor_data',
            executable='get_enc',
            name='get_enc'),
    ])