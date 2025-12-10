import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        #Main function that runs the angle based control node
        launch_ros.actions.Node(
            package='sensor_data',
            executable='UKF',
            name='UKF'),
        #Node that runs controller
        launch_ros.actions.Node(
            package='sensor_data',
            executable='get_IMU',
            name='get_IMU'),
        #Node that publishes encoder data
        launch_ros.actions.Node(
            package='sensor_data',
            executable='get_enc',
            name='get_enc'),

    ])