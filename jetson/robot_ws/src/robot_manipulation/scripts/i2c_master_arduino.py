#!/usr/bin/env python3
import rospy, struct, time
import hid
from std_msgs.msg import Float32MultiArray

# IDs do CP2112
VENDOR_ID = 0x10C4
PRODUCT_ID = 0xEA90

ARDUINO_ADDRESS = 0x0a  # Endereço I2C do Arduino
REPEATS = 4
INTERVAL = 0.5

class I2C_ROS_Bridge:
    def __init__(self):
        rospy.init_node('i2c_master_arduino', anonymous=True)

        # Inicializa CP2112
        self.dev = hid.Device(vid=VENDOR_ID, pid=PRODUCT_ID)

        self.arm = [0.0, 0.0]
        self.ee  = [0.0, 0.0]
        self.repeats = 0
        self.last_tx = 0.0

        rospy.Subscriber('/arm_control', Float32MultiArray, self.cb_arm, queue_size=1)
        rospy.Subscriber('/end_effector_control', Float32MultiArray, self.cb_ee,  queue_size=1)

    def cb_arm(self, msg):
        if len(msg.data) >= 2 and list(msg.data[:2]) != self.arm:
            self.arm = list(msg.data[:2])
            self.repeats = REPEATS

    def cb_ee(self, msg):
        if len(msg.data) >= 2 and list(msg.data[:2]) != self.ee:
            self.ee = list(msg.data[:2])
            self.repeats = REPEATS

    def send_once(self):
        # Empacota os 4 floats em bytes
        payload = struct.pack('<4f', *(self.arm + self.ee))

        # Monta report 0x01 para escrever no Arduino
        report = bytearray(64)
        report[0] = 0x01  # Report ID: Write
        report[1] = (ARDUINO_ADDRESS << 1)  # I2C address shifted
        report[2] = len(payload)  # Quantidade de bytes
        report[3] = 0x00  # SMBus flags
        report[4:8] = (5000).to_bytes(4, 'little')  # Timeout (5000 ms)
        report[8:8+len(payload)] = payload  # Dados

        # Converte para bytes e envia
        self.dev.write(bytes(report))


    def run(self):
        r = rospy.Rate(200)
        while not rospy.is_shutdown():
            if self.repeats and time.monotonic() - self.last_tx >= INTERVAL:
                self.send_once()
                self.last_tx = time.monotonic()
                self.repeats -= 1
            r.sleep()

        self.dev.close()

if __name__ == '__main__':
    try:
        I2C_ROS_Bridge().run()
    except rospy.ROSInterruptException:
        pass
