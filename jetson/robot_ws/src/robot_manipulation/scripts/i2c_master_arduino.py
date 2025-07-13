#!/usr/bin/env python3
import rospy, struct, time
from smbus2 import SMBus, i2c_msg
from std_msgs.msg import Float32MultiArray

I2C_BUS = 1
ARDUINO_ADDRESS = 0x0a
REPEATS = 4
INTERVAL = 0.5

class I2C_ROS_Bridge:
    def __init__(self):
        rospy.init_node('i2c_master_arduino', anonymous=True)
        self.bus = SMBus(I2C_BUS)
        self.arm = [0.0, 0.0]
        self.ee  = [0.0, 0.0]
        self.repeats = 0
        self.last_tx = 0.0
        rospy.Subscriber('/arm_control',          Float32MultiArray, self.cb_arm, queue_size=1)
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
        pkt = struct.pack('<4f', *(self.arm + self.ee))
        self.bus.i2c_rdwr(i2c_msg.write(ARDUINO_ADDRESS, pkt))

    def run(self):
        r = rospy.Rate(200)
        while not rospy.is_shutdown():
            if self.repeats and time.monotonic() - self.last_tx >= INTERVAL:
                self.send_once()
                self.last_tx = time.monotonic()
                self.repeats -= 1
            r.sleep()
        self.bus.close()

if __name__ == '__main__':
    try:
        I2C_ROS_Bridge().run()
    except rospy.ROSInterruptException:
        pass