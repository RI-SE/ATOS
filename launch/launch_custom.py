import sys
import os
from ament_index_python.packages import get_package_prefix
sys.path.insert(0, os.path.join( # Need to modify the sys.path since we launch from the ros2 installed path
    get_package_prefix('atos'),
    'share', 'atos', 'launch'))
import launch_utils.launch_base as launch_base
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
import argparse

def get_command_line_arguments():
# parse arguments from command line
	parser = argparse.ArgumentParser(description='Launch ATOS nodes')
	parser.add_argument('-h', '--help', action='help', help='Show this help message and exit')
	parser.add_argument('-g', '--graphical', action='store_true', help='Launch graphical nodes')
	parser.add_argument('-e', '--experimental', action='store_true', help='Launch experimental nodes')
	args = parser.parse_args()
	print(args.accumulate(args.integers))
	return args



def generate_launch_description():
	args = get_command_line_arguments

	ld = LaunchDescription()

	launch_graphical = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(get_package_share_directory('atos'), 'launch', 'launch_graphical.py')
		)
	)

	ld.add_action(launch_graphical)
	# ld.add_action(launch_experimental)
	return ld