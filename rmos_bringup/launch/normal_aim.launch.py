from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    daheng_node_l = Node(
        package='rmos_cam',
        executable='daheng_camera',
        namespace='rmos_cam_l',
        name='daheng_camera_l',
        parameters=[{'is_left': True}],
        output='screen',
    )

    daheng_node_r = Node(
        package='rmos_cam',
        executable='daheng_camera',
        namespace='rmos_cam_r',
        name='daheng_camera_r',
        parameters=[{'is_left': False}],
        output='screen',
    )

    basic_armor_detector_node_l = Node(
        package='rmos_detector',
        executable='basic_detector',
        namespace='rmos_detector_l',
        name='basic_detector_l',
        parameters=[{'is_left': True}],
        output='screen',
    )

    basic_armor_detector_node_r = Node(
        package='rmos_detector',
        executable='basic_detector',
        namespace='rmos_detector_r',
        name='basic_detector_r',
        parameters=[{'is_left': False}],
        output='screen',
    )

    processer_node_l = Node(
        package='rmos_processer',
        executable='processer',
        namespace='rmos_processer_l',
        name='processer_l',
        parameters=[{'is_left': True}],
        output='screen',
    )

    processer_node_r = Node(
        package='rmos_processer',
        executable='processer',
        namespace='rmos_processer_r',
        name='processer_r',
        parameters=[{'is_left': False}],
        output='screen',
    )

    communicate_node_l = Node(
        package='rmos_transporter',
        executable='usb_comm',
        namespace='rmos_transporter_l',
        name='usb_comm_l',
        parameters=[{'is_left': True}],
        output='screen',
    )

    communicate_node_r = Node(
        package='rmos_transporter',
        executable='usb_comm',
        namespace='rmos_transporter_r',
        name='usb_comm_r',
        parameters=[{'is_left': False}],
        output='screen',
    )

    recorder_node_l = Node(
        package='rmos_recorder',
        executable='recorder',
        namespace= 'rmos_recorder_l',
        name='recorder_l',
        parameters=[{'is_left': True, 'record_topic': True, 'record_log': True}],
        output='screen',
    )

    recorder_node_r = Node(
        package='rmos_recorder',
        executable='recorder',
        namespace= 'rmos_recorder_r',
        name='recorder_r',
        parameters=[{'is_left': False, 'record_topic': True, 'record_log': True}],
        output='screen',
    )

    return LaunchDescription([
        daheng_node_l,
        daheng_node_r,
        basic_armor_detector_node_l,
        basic_armor_detector_node_r,
        communicate_node_l,
        communicate_node_r,
        processer_node_l,
        processer_node_r,
        # recorder_node_l,
        # recorder_node_r,
    ])
