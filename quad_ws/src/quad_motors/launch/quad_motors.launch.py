from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    servo_parameters_path = str(get_package_share_path('quad_motors') / 'config' / 'servo_parameters.yaml')         
           
    quad_motors_node=Node(
        package = 'quad_motors',
        #name = 'quad_motors_node',
        executable = 'quad_motors',
        output='screen',  
        parameters = [{"servo_parameters_path": servo_parameters_path}])    
       
    ld.add_action(quad_motors_node)
   
    return ld    
