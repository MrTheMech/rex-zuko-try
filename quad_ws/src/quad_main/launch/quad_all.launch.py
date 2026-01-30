from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def get_source_config_path(package_name, config_file):
    """Get config path from source directory, with fallback to install directory."""
    # Try to find the workspace root by looking for common workspace markers
    
    # Method 1: Check common workspace locations
    possible_roots = [
        os.path.expanduser('~/rex-zuko-try/quad_ws'),
        os.path.expanduser('~/quad_ws'),
        '/home/rexbot1/rex-zuko-try/quad_ws',
    ]
    
    for ws_root in possible_roots:
        source_path = os.path.join(ws_root, 'src', package_name, 'config', config_file)
        if os.path.exists(source_path):
            return source_path
    
    # Method 2: Try to derive from install path
    install_path = str(get_package_share_path(package_name))
    if 'install' in install_path:
        ws_root = install_path.split('install')[0]
        source_path = os.path.join(ws_root, 'src', package_name, 'config', config_file)
        if os.path.exists(source_path):
            return source_path
    
    # Fallback to install directory
    return str(get_package_share_path(package_name) / 'config' / config_file)

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get config paths from SOURCE directory so changes take effect immediately
    motion_parameters_path = get_source_config_path('quad_main', 'motion_parameters.yaml')
    frame_parameters_path = get_source_config_path('quad_main', 'frame_parameters.yaml')
    linked_leg_parameters_path = get_source_config_path('quad_main', 'linked_leg_parameters.yaml')
        
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
        parameters=[{"joystick_number": 1}])  
    
    quad_simulation=Node(
        package = 'quad_simulation',
        # name = 'quad_node',
        executable = 'quad_simulation',
        output='screen')    

    ld.add_action(quad_main)
    ld.add_action(quad_gamepad)
    ld.add_action(quad_simulation)
    return ld    
