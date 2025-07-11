#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math

def deg_norm(a):
    return (a + 180.0) % 360.0 - 180.0

class StableYawPublisher:
    def __init__(self):
        #self.stable_yaw_deg = None     
        #self.threshold_deg = 1     
        self.stable_quat_z = None
        self.last_quat_z = None
        self.threshold_quat_z   = 0.001   

        self.pub = rospy.Publisher("/imu/data_stable", Imu, queue_size=10)
        rospy.Subscriber("/imu/data_filtered", Imu, self.imu_callback)

    def imu_callback(self, msg):
        q = msg.orientation
        #roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        #yaw_deg = math.degrees(yaw)

        if self.stable_quat_z is None:
            self.stable_quat_z = q.z
            self.last_quat_z = q.z

        delta = deg_norm(q.z - self.last_quat_z)

        if abs(delta) > self.threshold_quat_z: # Considerando apenas variações que sejam grandes o suficiente para considerar-se rotação real robô
            self.stable_quat_z += delta
        self.last_quat_z = q.z

        new_q = q
        new_q.z = self.stable_quat_z

        imu_out = Imu()
        imu_out.header = msg.header
        imu_out.orientation.x, imu_out.orientation.y, imu_out.orientation.z, imu_out.orientation.w = new_q
        imu_out.orientation_covariance         = msg.orientation_covariance
        imu_out.angular_velocity               = msg.angular_velocity
        imu_out.angular_velocity_covariance    = msg.angular_velocity_covariance
        imu_out.linear_acceleration            = msg.linear_acceleration
        imu_out.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.pub.publish(imu_out)

        # rospy.loginfo(
        #     "Δ={:+.2f}° | raw={:+.2f}° | pub={:+.2f}°".format(delta, yaw_deg, self.stable_yaw_deg)
        # )

def main():
    rospy.init_node("imu_stable")
    StableYawPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()
