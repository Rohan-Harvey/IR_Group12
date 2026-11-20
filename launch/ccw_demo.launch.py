from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    share = Path(get_package_share_directory('parking_lot_sim'))
    return LaunchDescription([DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']), DeclareLaunchArgument('report_dir', default_value=str(Path.cwd() / 'parking_results')), IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/parking_lot.launch.py')), launch_arguments={'autonomy': 'true', 'gui': LaunchConfiguration('gui'), 'report_dir': LaunchConfiguration('report_dir')}.items())])
