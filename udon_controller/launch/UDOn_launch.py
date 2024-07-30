from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    launch_imu_part = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('imu_filter_madgwick'), '/launch/imu_filter.launch.py'])
    )

    launch_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('udon_controller'), '/launch/ekf.launch.py'])
    )

    launch_rplidar_part = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('rplidar_ros'), '/launch/rplidar_a1_launch.py'])
    )

    run_realsense = Node(
        package='realsense_cam',
        executable='camera_on_node',
        name = 'realsense_cam',
        output='screen'
    )

    run_arm = Node(
        package='arm_control_pkg',
        executable='arm_control_node',
        name='arm_control_node',
        output='screen'
    )

    # launch_arm_part = IncludeLaunchDescription(
    #     AnyLaunchDescriptionSource([get_package_share_directory('arm_control_pkg'), '/launch/arm_launch.xml'])
    # )

    return LaunchDescription([
        launch_imu_part,
        launch_ekf,
        launch_rplidar_part,
        # launch_arm_part,
        run_realsense,
        run_arm
    ])