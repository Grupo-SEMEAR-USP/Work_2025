#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def deg_norm(a):
    return (a + 180.0) % 360.0 - 180.0

def angle_correction(a):
    if (a < 0):
        return (a+360)
    return a

class StableYawPublisher:
    def __init__(self):
        self.stable_yaw_deg = None     
        self.last_yaw_deg = None
        self.threshold_deg = 0.01
        self.offset_deg = 0.0

        self.pub = rospy.Publisher("/imu/data_stable", Imu, queue_size=10)
        rospy.Subscriber("/imu/data_filtered", Imu, self.imu_callback)

    def imu_callback(self, msg):
        q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw_deg = math.degrees(yaw)
        #yaw_deg = angle_correction(yaw_deg)

        if self.stable_yaw_deg is None:
            self.offset_deg = yaw_deg
            self.stable_yaw_deg = 0.0
            self.last_yaw_deg = yaw_deg

        delta = deg_norm(yaw_deg - self.last_yaw_deg)

        if abs(delta) > self.threshold_deg:
            self.stable_yaw_deg -= delta
            #self.stable_yaw_deg = yaw_deg

        self.last_yaw_deg = yaw_deg

        print(self.stable_yaw_deg)

        # Converte o yaw filtrado de volta para quaternion
        stable_yaw_rad = math.radians(self.stable_yaw_deg)
        new_q = quaternion_from_euler(0, 0, stable_yaw_rad)

        imu_out = Imu()
        imu_out.header = msg.header
        imu_out.orientation.x = new_q[0]
        imu_out.orientation.y = new_q[1]
        imu_out.orientation.z = new_q[2]
        imu_out.orientation.w = new_q[3]
        imu_out.orientation_covariance         = msg.orientation_covariance
        imu_out.angular_velocity               = msg.angular_velocity
        imu_out.angular_velocity_covariance    = msg.angular_velocity_covariance
        imu_out.linear_acceleration            = msg.linear_acceleration
        imu_out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(imu_out)

def main():
    rospy.init_node("imu_stable")
    StableYawPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
