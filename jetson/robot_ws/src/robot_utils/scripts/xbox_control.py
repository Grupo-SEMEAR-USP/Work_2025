#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray

DEADZONE = 0.05

class XboxTeleop:
    def __init__(self):
        self.arm_pub = rospy.Publisher('/arm_control', Float32MultiArray, queue_size=1)
        self.ee_pub  = rospy.Publisher('/end_effector_control', Float32MultiArray, queue_size=1)
        rospy.Subscriber('/joy', Joy, self.joy_cb, queue_size=1)
        self.last_arm = [0.0, 0.0]
        self.last_ee  = [0.0, 0.0]
        self.scale = rospy.get_param('~scale', 1.0)

        self.wrist_value = 0.0
        self.grip_value = 0.0

        self.y_last = 0
        self.a_last = 0

    def joy_cb(self, msg):
        elev  = self.scale * self._dz(msg.axes[1])
        base  = self.scale * self._dz(msg.axes[0]) 

        # B sets wrist = 90
        if msg.buttons[1]:
            self.grip_value = 80.0
        # X sets wrist = 0
        elif msg.buttons[2]:
            self.grip_value = 0.0

        # Y increments +10 (edge triggered)
        y_pressed = msg.buttons[3]
        if y_pressed and not self.y_last:
            self.wrist_value += 10.0
        self.y_last = y_pressed

        # A decrements -10 (edge triggered)
        a_pressed = msg.buttons[0]
        if a_pressed and not self.a_last:
            self.wrist_value -= 10.0
        self.a_last = a_pressed

        arm = [elev, base]
        ee  = [self.grip_value, self.wrist_value]

        if arm != self.last_arm:
            self.arm_pub.publish(Float32MultiArray(data=arm))
            self.last_arm = arm
        if ee != self.last_ee:
            self.ee_pub.publish(Float32MultiArray(data=ee))
            self.last_ee = ee

    @staticmethod
    def _dz(v):
        return 0.0 if abs(v) < DEADZONE else v

if __name__ == '__main__':
    rospy.init_node('xbox_teleop')
    XboxTeleop()
    rospy.spin()
