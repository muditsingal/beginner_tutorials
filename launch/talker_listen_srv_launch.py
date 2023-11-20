from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='beginner_tutorials',
      namespace='beginner_tutorials_ns',
      executable='talker',
      name='taker_node'
    ),
    Node(
      package='beginner_tutorials',
      namespace='beginner_tutorials_ns',
      executable='listener',
      name='listener_node'
    ),
    Node(
      package='beginner_tutorials',
      namespace='beginner_tutorials_ns',
      executable='server',
      name='server_node'
    )
  ])
