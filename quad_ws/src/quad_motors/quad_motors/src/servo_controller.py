from time import sleep
from math import pi
from src.PCA9685Servos import PCA9685Servos

"""
PCA9685 Pin Layout:
- PIN0:  FRT (Front Right Tibia)     - Joint 5
- PIN1:  FLT (Front Left Tibia)      - Joint 2
- PIN2:  FRF (Front Right Femur)     - Joint 4
- PIN3:  FLF (Front Left Femur)      - Joint 1
- PIN4:  FL  (Front Left Hip)        - Joint 0
- PIN5:  FR  (Front Right Hip)        - Joint 3
- PIN6:  BRF (Back Right Femur)      - Joint 10
- PIN7:  BLF (Back Left Femur)       - Joint 7
- PIN8:  BRT (Back Right Tibia)      - Joint 11
- PIN9:  BLT (Back Left Tibia)       - Joint 8
- PIN10: BL  (Back Left Hip)         - Joint 6
- PIN11: BR  (Back Right Hip)        - Joint 9

Joint Index Order (12 joints total):
- Joints 0-2:   FL (Hip, Femur, Tibia)
- Joints 3-5:   FR (Hip, Femur, Tibia)
- Joints 6-8:   BL (Hip, Femur, Tibia)
- Joints 9-11:  BR (Hip, Femur, Tibia)
"""


class ServoController():
    def __init__(self, bus_index, device_address, servo_parameters):
        self.servo_parameters = servo_parameters
        self.servo_driver = PCA9685Servos(bus_index, device_address)

    def clamp(self, num, min_val, max_val):
        return (max(min(num, max_val), min_val))

    def set_servo_angles(self, servo_angles):
        for i in range(12): 
            zero_degrees_pulse_width = self.servo_parameters['zero_degrees_pulse_width'][i]
            pulse_width_per_degree = self.servo_parameters['pulse_width_per_degree'][i]
            invert_direction = self.servo_parameters['invert_direction'][i]
            min_degrees = self.servo_parameters['min_degrees'][i]
            max_degrees = self.servo_parameters['max_degrees'][i]
            hardware_pin = self.servo_parameters['map_joint_index_to_driver_pin'][i]

            angle = servo_angles[i] * (180 / pi)

            if invert_direction:
                angle = -angle

            angle = self.clamp(angle, min_degrees, max_degrees)
                
            pulse_width = angle * pulse_width_per_degree + zero_degrees_pulse_width

            self.servo_driver.set_pulse_width(hardware_pin, int(pulse_width))


