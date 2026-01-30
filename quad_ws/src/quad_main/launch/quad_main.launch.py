from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def get_source_config_path(launch_file_dir, package_name, config_file):
    """Get config path from source directory, with fallback to install directory."""
    # Navigate from quad_main/launch to workspace src directory
    workspace_src = os.path.realpath(os.path.join(launch_file_dir, '..', '..'))
    source_path = os.path.join(workspace_src, package_name, 'config', config_file)
    
    if os.path.exists(source_path):
        return source_path
    # Fallback to install directory for deployed systems
    return str(get_package_share_path(package_name) / 'config' / config_file)

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get config paths from SOURCE directory so changes take effect immediately
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    
    motion_parameters_path = get_source_config_path(launch_file_dir, 'quad_main', 'motion_parameters.yaml')
    frame_parameters_path = get_source_config_path(launch_file_dir, 'quad_main', 'frame_parameters.yaml')
    linked_leg_parameters_path = get_source_config_path(launch_file_dir, 'quad_main', 'linked_leg_parameters.yaml')
        
    quad_main=Node(
        package = 'quad_main',
        #name = 'quad_main',
        executable = 'quad_main',
        output='screen',  
        parameters = [{"motion_parameters_path": motion_parameters_path}, {"frame_parameters_path": frame_parameters_path},{"linked_leg_parameters_path": linked_leg_parameters_path}])
    
    quad_gamepad=Node(
        package = 'quad_gamepad',
        # name = 'quad_node',
        executable = 'quad_gamepad',
        output='screen',
        parameters=[{"joystick_number": 2}]        
    )      
    ld.add_action(quad_main)
    ld.add_action(quad_gamepad)
    return ld    
