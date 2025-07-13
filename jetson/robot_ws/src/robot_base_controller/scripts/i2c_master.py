#!/usr/bin/env python3
import rospy
import smbus
import struct
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from robot_base_controller.msg import encoder_data, velocity_data
from time import sleep
import threading

# Constantes
ESP32_ADDRESS_FRONT = 0x09 
ESP32_ADDRESS_REAR = 0x08
I2C_BUS = 1  
REG_ADDRESS = 10 

encoder_data_global = [0, 0, 0, 0]

# Classe para comunicação I2C
class I2CCommunication:
    def __init__(self, device_address):

        self.wheel_velocities = [0, 0]

        if device_address == ESP32_ADDRESS_FRONT:
            self.wheel_indices = 0 # Indice front
        elif device_address == ESP32_ADDRESS_REAR:
            self.wheel_indices = 1  # Indice rears

        self.i2c = smbus.SMBus(I2C_BUS)  # Define o barramento que será usado na comunicação
        self.device_address = device_address  # Define o endereço da ESP32 ao qual queremos nos comunicar

        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_joints = rospy.Subscriber('/velocity_command', velocity_data, self.joints_callback)

        self.encoder_msg = encoder_data()

        if device_address == ESP32_ADDRESS_FRONT:
            self.encoder_msg.front_left_encoder_data = 0
            self.encoder_msg.front_right_encoder_data = 0
        else:
            self.encoder_msg.rear_left_encoder_data = 0
            self.encoder_msg.rear_right_encoder_data = 0

        self.thread = threading.Thread(target=self.update)
        self.thread.start() 

    def read_data(self):
        try:
            # Lê 9 bytes do buffer da ESP32
            data = self.i2c.read_i2c_block_data(self.device_address, REG_ADDRESS, 9)

            if data[0] != 0x01:
                return
            
            # Ignora o header (data[0])
            value_right = struct.unpack('!i', bytes(data[1:5]))
            value_left = struct.unpack('!i', bytes(data[5:9]))  
            
            if self.wheel_indices == 0:
                encoder_data_global[0] = value_left[0]
                encoder_data_global[1] = value_right[0]
            else:
                encoder_data_global[2] = value_left[0]
                encoder_data_global[3] = value_right[0]

            self.encoder_msg.front_left_encoder_data = (encoder_data_global[0] / 1000)
            self.encoder_msg.front_right_encoder_data = (encoder_data_global[1] / 1000) 
            self.encoder_msg.rear_left_encoder_data = (encoder_data_global[3] / 1000) 
            self.encoder_msg.rear_right_encoder_data = (encoder_data_global[2] / 1000) 

            self.pub_encoder.publish(self.encoder_msg)

        except Exception as e:
            # rospy.logerr(f"Erro na leitura: {str(e)}")
            return None


    def write_data(self):

        try:
            data = struct.pack('!ii', self.wheel_velocities[0], self.wheel_velocities[1])
            self.i2c.write_i2c_block_data(self.device_address, 0, list(data))

            # rospy.loginfo(f'Valores enviados: {self.wheel_velocities}')

        except Exception as e:
            rospy.logerr(f"Erro na escrita: {str(e)}")
            return None

    def joints_callback(self, msg):

        if msg is not None:
            if self.wheel_indices == 0:
                self.wheel_velocities[0] = int(msg.front_left_wheel * 1000)
                self.wheel_velocities[1] = int(msg.front_right_wheel * 1000)
            else:
                self.wheel_velocities[0] = int(msg.rear_left_wheel * 1000)
                self.wheel_velocities[1] = int(msg.rear_right_wheel * 1000)

    def update(self):
        rate = rospy.Rate(30)  # 10Hz

        while not rospy.is_shutdown():

            self.read_data()
            self.write_data()
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('i2c_master', anonymous=True)
        left_i2c_communication = I2CCommunication(ESP32_ADDRESS_REAR)
        right_i2c_communication = I2CCommunication(ESP32_ADDRESS_FRONT)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass