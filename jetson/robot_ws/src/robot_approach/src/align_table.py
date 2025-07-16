#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32
from robot_scheduler.msg import SchedulerCommand
from robot_perception.msg import UltrasonicDistances

TARGET_NAME = "align"

class MoveToTarget:
    def __init__(self):
        rospy.loginfo("[align_table] Inicializado e aguardando comandos...")
        
        rospy.init_node('move_to_target', anonymous=True)
        rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)
        rospy.Subscriber("/ultrasonic_distances", UltrasonicDistances, self.ultrasound_callback, queue_size=10)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        self.dist_right = -1
        self.dist_left = -1
        self.diff = -1
        self.dist_min = -1    # Distância para o robo parar de se aproximar da mesa
        self.diff_max = 3.0     # Diferença máxima aceitável entre os dois ultrassonicos
        self.flag_align = False
        self.flag_getCloser = False
        self.lastID = None


    def sched_cb(self, msg):
        if msg.target != TARGET_NAME or msg.uid == self.lastID:
            return
        
        rospy.loginfo(f"[align_table] Comando para iniciar a aproximação recebido!")
        self.lastID = msg.uid
        self.dist_min = float(msg.payload) # eh isso?
        self.flag_align = True

    def ultrasound_callback(self, msg):
        self.dist_left = msg.front_left
        self.dist_right = msg.front_right
        self.diff = self.dist_right - self.dist_left
        
        if self.flag_align:
            self.align()
            
        elif self.flag_getCloser:
            self.get_closer()


    # Alinha com a mesa
    def align(self):
        rospy.loginfo("[align_table] Distância dos ultrassônicos: %.2f (esq) %.2f (dir)", self.dist_left, self.dist_right)

        twist = Twist()

        # if self.dist_right == -1 and self.dist_left == -1:
            # algoritmo de busca ?

        # vai pra esquerda
        if self.dist_right == -1 or self.diff > self.diff_max:
            twist.angular.z = 0.5
            
        # vai pra direita
        elif self.dist_left == -1 or self.diff < -self.diff_max:
            twist.angular.z = -0.5
        
        elif abs(self.diff) <= self.diff_max:
            rospy.loginfo("[align_table] Robô alinhado")
            self.flag_align = False

            if min(self.dist_left, self.dist_right) > 10:
                self.flag_getCloser = True
                rospy.loginfo("[align_table] Se aproximando do robô")
            
            twist.angular.z = 0

        self.pub_vel.publish(self.twist)

    # Se aproxima da mesa
    def get_closer(self):
        dist = min(self.dist_left, self.dist_right)
        twist = Twist()

        if dist > self.dist_min:
            twist.linear.x = 0.5
        else:
            twist.linear.x = 0.0
            rospy.loginfo("[align_table] Robô está próximo da mesa")

            # alinha novamente se necessário
            if abs(self.diff) > self.diff_max:
                self.flag_align = True

        self.pub_vel.publish(twist)


if __name__ == '__main__':
    try:
        move_to_target = MoveToTarget()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
