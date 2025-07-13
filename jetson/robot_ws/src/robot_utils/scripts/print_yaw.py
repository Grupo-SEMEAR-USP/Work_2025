#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def imu_callback(msg):
    q = msg.orientation
    quaternion = [q.x, q.y, q.z, q.w]

    roll, pitch, yaw = euler_from_quaternion(quaternion)

    yaw_deg = math.degrees(yaw)
    rospy.loginfo("Yaw: {:.5f} degrees".format(yaw_deg))

def main():
    rospy.init_node("print_yaw_node")
    rospy.Subscriber("/imu/data_stable", Imu, imu_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
