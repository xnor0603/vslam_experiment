import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 1. 定義模擬時間參數
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (bag) clock if true'
    )

    # 2. VSLAM 節點配置
    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
            'use_sim_time': use_sim_time,
            'enable_imu_fusion': True,
            'enable_rectified_pose': True,
            'denoise_input_images': False,
            'rectified_images_l_topic': 'visual_slam/left_image',
            'rectified_images_r_topic': 'visual_slam/right_image',
            'input_imu_topic': 'visual_slam/imu',
            'base_frame': 'base_link',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'num_cameras': 2,
            'camera_optical_frames': [
                'camera_infra1_optical_frame',
                'camera_infra2_optical_frame',
            ],
            'imu_frame': 'camera_imu_optical_frame',
            'input_left_camera_info_topic': 'visual_slam/left_camera_info',
            'input_right_camera_info_topic': 'visual_slam/right_camera_info'
        }],
        remappings=[
            ('visual_slam/image_0', '/camera/camera/infra1/image_rect_raw'),
            ('visual_slam/image_1', '/camera/camera/infra2/image_rect_raw'),
            ('visual_slam/camera_info_0', '/camera/camera/infra1/camera_info'),
            ('visual_slam/camera_info_1', '/camera/camera/infra2/camera_info'),
            ('visual_slam/imu', '/camera/camera/imu')
        ]
    )

    # 3. 靜態座標變換發布器（bag 重播模式：RealSense 驅動沒在跑，TF 鏈要自己補）
    #
    # 鏈：base_link → camera_link → camera_infra{1,2}_optical_frame / camera_imu_optical_frame
    #
    # 數值來源：realsense2_description/urdf/_d435.urdf.xacro 與 _d435i_imu_modules.urdf.xacro
    #   - infra1: y=0,       optical RPY=(-π/2, 0, -π/2)
    #   - infra2: y=-0.050,  optical RPY=(-π/2, 0, -π/2)   ← 50mm 立體基線
    #   - imu   : (-0.01174, -0.00552, 0.0051), optical RPY=(-π/2, 0, -π/2)
    # 為「nominal extrinsics」近似（與 EEPROM 標定有 mm 級差異），cuVSLAM 對此通常 robust。
    # 參數順序：x y z yaw pitch roll parent child
    PI_2 = '-1.5707963267948966'
    tf_base_to_camera_node = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='base_link_to_camera_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[{'use_sim_time': use_sim_time}])

    tf_camera_to_infra1 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_link_to_infra1_optical_tf',
        arguments=['0', '0', '0', PI_2, '0', PI_2, 'camera_link', 'camera_infra1_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    tf_camera_to_infra2 = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_link_to_infra2_optical_tf',
        arguments=['0', '-0.050', '0', PI_2, '0', PI_2, 'camera_link', 'camera_infra2_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    tf_camera_to_imu = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='camera_link_to_imu_optical_tf',
        arguments=['-0.01174', '-0.00552', '0.0051', PI_2, '0', PI_2, 'camera_link', 'camera_imu_optical_frame'],
        parameters=[{'use_sim_time': use_sim_time}])

    # 4. 容器配置
    visual_slam_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[visual_slam_node],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        tf_base_to_camera_node,
        tf_camera_to_infra1,
        tf_camera_to_infra2,
        tf_camera_to_imu,
        visual_slam_container
    ])