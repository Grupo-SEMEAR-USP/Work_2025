#!/usr/bin/env python3
import rospy
import hid
import struct
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from robot_base_controller.msg import encoder_data, velocity_data
from time import sleep
import threading

# IDs do CP2112
VENDOR_ID = 0x10C4
PRODUCT_ID = 0xEA90

# Constantes
ESP32_ADDRESS_FRONT = 0x08
ESP32_ADDRESS_REAR = 0x09
REG_ADDRESS = 10

encoder_data_global = [0, 0, 0, 0]

class I2CCommunication:
    def __init__(self, device_address):
        self.wheel_velocities = [0, 0]

        if device_address == ESP32_ADDRESS_FRONT:
            self.wheel_indices = 0
        elif device_address == ESP32_ADDRESS_REAR:
            self.wheel_indices = 1

        # Inicializa CP2112
        self.dev = hid.Device(vid=VENDOR_ID, pid=PRODUCT_ID)

        self.device_address = device_address

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
            # Primeiro, enviar comando de registrador (se seu ESP32 espera um byte de registro)
            write_report = bytearray(64)
            write_report[0] = 0x01  # Report ID
            write_report[1] = (self.device_address << 1)
            write_report[2] = 1
            write_report[3] = 0x00
            write_report[4:8] = (5000).to_bytes(4, 'little')
            write_report[8] = REG_ADDRESS
            self.dev.write(write_report)

            # Em seguida, enviar comando de leitura
            read_report = bytearray(64)
            read_report[0] = 0x02
            read_report[1] = (self.device_address << 1)
            read_report[2] = 9
            read_report[3] = 0x00
            read_report[4:8] = (5000).to_bytes(4, 'little')
            self.dev.write(read_report)

            # Receber resposta
            response = self.dev.read(64)
            if not response or response[0] != 0x03:
                return

            data = response[2:11]  # Dados começam no índice 2

            if data[0] != 0x01:
                return

            value_right = struct.unpack('!i', bytes(data[1:5]))
            value_left = struct.unpack('!i', bytes(data[5:9]))

            if self.wheel_indices == 0:
                encoder_data_global[0] = value_left[0]
                encoder_data_global[1] = value_right[0]
            else:
                encoder_data_global[2] = value_left[0]
                encoder_data_global[3] = value_right[0]

            self.encoder_msg.front_left_encoder_data = (encoder_data_global[0] / 1000) * 0.118 * 0.10472
            self.encoder_msg.front_right_encoder_data = (encoder_data_global[1] / 1000) * 0.118 * 0.10472
            self.encoder_msg.rear_left_encoder_data = (encoder_data_global[2] / 1000) * 0.118 * 0.10472
            self.encoder_msg.rear_right_encoder_data = (encoder_data_global[3] / 1000) * 0.118 * 0.10472

            self.pub_encoder.publish(self.encoder_msg)

        except Exception as e:
            rospy.logerr(f"Erro na leitura: {str(e)}")
            return None

    def write_data(self):
        try:
            payload = bytearray(9)
            payload[0] = 0x01
            data = struct.pack('!ii', self.wheel_velocities[0], self.wheel_velocities[1])
            payload[1:] = data

            write_report = bytearray(64)
            write_report[0] = 0x01
            write_report[1] = (self.device_address << 1)
            write_report[2] = 9
            write_report[3] = 0x00
            write_report[4:8] = (5000).to_bytes(4, 'little')
            write_report[8:17] = payload

            self.dev.write(write_report)

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
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.read_data()
            self.write_data()
            rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node('i2c_master_cp2112', anonymous=True)
        left_i2c_communication = I2CCommunication(ESP32_ADDRESS_REAR)
        right_i2c_communication = I2CCommunication(ESP32_ADDRESS_FRONT)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
