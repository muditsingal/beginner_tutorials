from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution


def generate_launch_description():
  return LaunchDescription([
    Node(
      package='beginner_tutorials',
      executable='talker',
      name='taker_node'
    ),
    Node(
      package='beginner_tutorials',
      executable='listener',
      name='listener_node'
    ),
    DeclareLaunchArgument(
      'rosbag_record',
      default_value = TextSubstitution(text = "True"),
      choices = ['True', 'False'],
      description = "Argument to enable/disable recording of all ros2 topics"
    ),
    ExecuteProcess(
      condition=IfCondition(LaunchConfiguration('rosbag_record')),
      cmd=['ros2', 'bag', 'record', '-a'],
      shell=True
    )
  ])
