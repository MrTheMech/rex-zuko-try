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
    
    # Get servo config from SOURCE directory so calibration changes take effect immediately
    servo_parameters_path = get_source_config_path('quad_motors', 'servo_parameters.yaml')
           
    quad_motors_node=Node(
        package = 'quad_motors',
        #name = 'quad_motors_node',
        executable = 'quad_motors',
        output='screen',  
        parameters = [{"servo_parameters_path": servo_parameters_path}])    
       
    ld.add_action(quad_motors_node)
   
    return ld    
