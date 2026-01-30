from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Read from SOURCE directory so calibration changes take effect immediately
    # without needing to rebuild. The source path is relative to this launch file.
    launch_file_dir = os.path.dirname(os.path.realpath(__file__))
    source_config_path = os.path.join(launch_file_dir, '..', 'config', 'servo_parameters.yaml')
    servo_parameters_path = os.path.realpath(source_config_path)
    
    # Fallback to install directory if source not found (for deployed systems)
    if not os.path.exists(servo_parameters_path):
        servo_parameters_path = str(get_package_share_path('quad_motors') / 'config' / 'servo_parameters.yaml')
           
    quad_motors_node=Node(
        package = 'quad_motors',
        #name = 'quad_motors_node',
        executable = 'quad_motors',
        output='screen',  
        parameters = [{"servo_parameters_path": servo_parameters_path}])    
       
    ld.add_action(quad_motors_node)
   
    return ld    

