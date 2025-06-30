#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simula encoders (posição + velocidade) e odometria para o robô mecanum
definido no seu URDF.  NENHUM parâmetro externo é necessário.
"""
import rospy, numpy as np, tf2_ros, tf_conversions
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class SymEncoder:
    # ----- parâmetros físicos (batem com seu URDF) -----
    R   = 0.025          # raio da roda       [m]
    LX  = 0.075          # meia-distância X   [m]
    LY  = 0.075          # meia-distância Y   [m]
    L   = LX + LY        # soma usada na cinemática
    JN  = ['joint_wheel_front_left',
           'joint_wheel_front_right',
           'joint_wheel_rear_left',
           'joint_wheel_rear_right']

    def __init__(self):
        # estado dos encoders
        self.pos = np.zeros(4)   # rad
        self.vel = np.zeros(4)   # rad/s

        # pose integrada
        self.x = self.y = self.yaw = 0.0
        self.last = rospy.Time.now()

        # pubs / subs
        self.pub_js   = rospy.Publisher('/joint_states',        JointState, queue_size=10)
        self.pub_odom = rospy.Publisher('/wheel_odom',          Odometry,   queue_size=10)
        self.br       = tf2_ros.TransformBroadcaster()

        self.cmd = Twist()   # último /cmd_vel recebido
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_cb, queue_size=1)

        # loop de 50 Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def cmd_cb(self, msg):           # apenas guarda o último Twist
        self.cmd = msg

    # ----------- coração: actualiza estado e publica -----------
    def update(self):
        now = rospy.Time.now()
        dt  = (now - self.last).to_sec()
        if dt == 0: return
        self.last = now

        #  ▼▼▼  cinemática mecanum  ▼▼▼
        vx, vy, wz = self.cmd.linear.x, self.cmd.linear.y, self.cmd.angular.z
        self.vel[0] = (1/self.R)*( vx -  vy - self.L*wz)   # FL
        self.vel[1] = (1/self.R)*( vx +  vy + self.L*wz)   # FR
        self.vel[2] = (1/self.R)*( vx +  vy - self.L*wz)   # RL
        self.vel[3] = (1/self.R)*( vx -  vy + self.L*wz)   # RR
        self.pos   += self.vel*dt                         # integração simples
        #  ▲▲▲----------------------------------------------------

        # ---- /joint_states ----
        js = JointState()
        js.header.stamp = now
        js.name         = self.JN
        js.position     = self.pos.tolist()
        js.velocity     = self.vel.tolist()
        self.pub_js.publish(js)

        # ---- odometria integrada (opcional, mas útil p/ EKF) ----
        self.x   += (vx*np.cos(self.yaw) - vy*np.sin(self.yaw))*dt
        self.y   += (vx*np.sin(self.yaw) + vy*np.cos(self.yaw))*dt
        self.yaw += wz*dt

        odom                = Odometry()
        odom.header.stamp   = now
        odom.header.frame_id= 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0,0,self.yaw)
        odom.pose.pose.orientation = Quaternion(*q)
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.linear.y  = vy
        odom.twist.twist.angular.z = wz
        self.pub_odom.publish(odom)

        # ---- TF odom → base_link ----
        t = TransformStamped()
        t.header.stamp    = now
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation      = odom.pose.pose.orientation
        self.br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('sym_encoder')
    SymEncoder()
