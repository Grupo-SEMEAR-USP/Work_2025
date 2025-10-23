#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from robot_base_controller.msg import UltrasonicDistances
from collections import deque

# CONFORME ARRUMAR OS ULTRAS, PRECISAMOS MODIFICAR A LEITURA DOS MESMOS NA CALLBACK

TARGET_NAME = "table_approach"
BUFFER_SIZE = 5  # Número de leituras para média móvel

class MoveToTarget:
    def __init__(self):
        rospy.init_node('move_to_target', anonymous=True)
        rospy.loginfo("[table_approach] Inicializado e aguardando comandos...")

        rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)
        rospy.Subscriber("/ultrasonic_distances", UltrasonicDistances, self.ultrasound_callback, queue_size=10)
        
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)

        # Buffers de leitura
        self.buffer_left = deque(maxlen=BUFFER_SIZE)
        self.buffer_right = deque(maxlen=BUFFER_SIZE)

        self.dist_right = -1
        self.dist_left = -1
        self.diff = -1
        self.table_dist = -1  
        self.tolerance = 2  # tolerância de erro para a distância entre o robô e a mesa    
        self.diff_max = 2
        self.ang_vel = 0.05
        self.lin_vel = 0.01
        self.flag_align = False
        self.flag_fixDist = False
        self.flag_start = False
        self.flag_finished = False
        self.lastID = None

    def sched_cb(self, msg):
        if msg.target != TARGET_NAME or msg.uid == self.lastID:
            return
        rospy.loginfo(f"[table_approach] Comando para iniciar a aproximação recebido!")
        self.lastID = msg.uid
        self.table_dist = float(msg.payload)
        self.flag_start = True
        self.flag_align = True
        self.flag_finished = False

    def ultrasound_callback(self, msg):
        # Armazena no buffer
        self.buffer_left.append(msg.front_left)
        self.buffer_right.append(msg.front_right)

        # Calcula média apenas se buffer cheio
        if len(self.buffer_left) == BUFFER_SIZE and len(self.buffer_right) == BUFFER_SIZE:
            self.dist_left = sum(self.buffer_left) / BUFFER_SIZE
            self.dist_right = sum(self.buffer_right) / BUFFER_SIZE
            self.diff = self.dist_right - self.dist_left

    def publish_feedback(self):
        feedback = SchedulerResponse()
        feedback.uid = self.lastID
        feedback.target = TARGET_NAME
        feedback.status = True
        for _ in range(5):
            self.pub_feedback.publish(feedback)

    def align(self):
        rospy.loginfo("[table_approach] Distância dos ultrassônicos: %.2f (esq) %.2f (dir)", self.dist_left, self.dist_right)
        twist = Twist()

        if self.dist_right == -1 or self.diff > self.diff_max:
            twist.angular.z = self.ang_vel
            rospy.loginfo("[table_approach] Alinhando... Diferença: %.2f", self.diff)
        elif self.dist_left == -1 or self.diff < -self.diff_max:
            twist.angular.z = -self.ang_vel
            rospy.loginfo("[table_approach] Alinhando... Diferença: %.2f", self.diff)
        elif abs(self.diff) <= self.diff_max:
            rospy.loginfo("[table_approach] Robô alinhado")
            rospy.sleep(1)
            self.flag_align = False

            if self.flag_start and not self.check_distance():
                self.flag_start = False
                self.flag_fixDist = True
                rospy.loginfo("[table_approach] Ajustando distância com a mesa")
            else:
                self.flag_finished = True

            twist.angular.z = 0.0

        self.pub_vel.publish(twist)

    def check_distance(self):
        dist = min(self.dist_left, self.dist_right)     # verificação baseada na menor distância  

        return (dist <= self.table_dist + self.tolerance and 
                dist >= self.table_dist - self.tolerance)

    def fix_distance(self):
        dist = min(self.dist_left, self.dist_right)     # ajuste baseado na menor distância
        twist = Twist()

        if dist > self.table_dist + self.tolerance:
            twist.linear.x = self.lin_vel

        # elif dist < self.table_dist - self.tolerance:
        #     twist.linear.x = -self.lin_vel

        else:
            self.flag_fixDist = False
            twist.linear.x = 0.0
            rospy.loginfo("[table_approach] Robô está próximo da mesa")
            self.flag_finished = True
            # if abs(self.diff) > self.diff_max:
            #     self.flag_align = True
            # else:
            #     self.flag_finished = True
        
        self.pub_vel.publish(twist)

    def spin(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.flag_align:
                self.align()
            elif self.flag_fixDist:
                self.fix_distance()
            if self.flag_finished:
                self.publish_feedback()
                self.flag_align = False
                self.flag_fixDist = False
                self.flag_start = False
                self.flag_finished = False

            rate.sleep()


if __name__ == '__main__':
    try:
        move_to_target = MoveToTarget()
        move_to_target.spin()
    except rospy.ROSInterruptException:
        pass