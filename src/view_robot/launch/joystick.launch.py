from launch import LaunchDescription
from launch_ros.actions import Node

# ĐÂY LÀ HÀM BẮT BUỘC PHẢI CÓ TRONG MỌI FILE LAUNCH
def generate_launch_description():

    #linux_joy
    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': '/dev/input/js0', # cap quyen cho tay cam
            'deadzone': 0.1,         # Vùng chết để cần không bị trôi
            'autorepeat_rate': 20.0,
        }]
    )

    return LaunchDescription([
        joy_node,
    ])