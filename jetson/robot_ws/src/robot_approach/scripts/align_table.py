#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32, Float32
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from robot_base_controller.msg import UltrasonicDistances
from collections import deque

# CONFORME ARRUMAR OS ULTRAS, PRECISAMOS MODIFICAR A LEITURA DOS MESMOS NA CALLBACK

TARGET_NAME = "align_table"
BUFFER_SIZE = 5  # Número de leituras para média móvel

class MoveToTarget:
    def __init__(self):
        rospy.init_node('align_table', anonymous=True)
        rospy.loginfo("[align_table] Inicializado e aguardando comandos...")

        rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)
        rospy.Subscriber("/ultrasonic_distances", UltrasonicDistances, self.ultrasound_callback, queue_size=10)
        
        self.pub_move = rospy.Publisher("move_time", String, queue_size=10)
        self.pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)

        # Buffers de leitura
        self.buffer_left = deque(maxlen=BUFFER_SIZE)
        self.buffer_right = deque(maxlen=BUFFER_SIZE)

        self.just_align = True
        self.dist_right = -1
        self.dist_left = -1
        self.diff = -1
        self.table_dist = -1  
        self.tolerance = 1  # tolerância de erro para a distância entre o robô e a mesa    
        self.diff_max = 0.5
        self.ang_vel = 0.1
        self.lin_vel = 0.05
        self.flag_align = False
        self.flag_fixDist = False
        self.flag_start = False
        self.flag_finished = False
        self.lastID = None
        self.trade = None
        self.timeout = None
        self.startTime = None

    def sched_cb(self, msg):
        if msg.target != TARGET_NAME or msg.uid == self.lastID:
            return
        rospy.loginfo(f"[align_table] Comando para iniciar a aproximação recebido!")
        self.lastID = msg.uid
        self.table_dist = float(msg.payload)
        self.flag_start = True
        self.flag_align = True
        self.flag_finished = False
        self.timeout = msg.timeout
        self.startTime = rospy.Time.now()

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
        if self.flag_finished:
            feedback.status = "OK"
        else:
            feedback.status = "FAIL"
        for _ in range(5):
            self.pub_feedback.publish(feedback)
        rospy.sleep(1)

    def align(self):
        # rospy.loginfo("[align_table] Distância dos ultrassônicos: %.2f (esq) %.2f (dir)", self.dist_left, self.dist_right)
        twist = Twist()
        if (rospy.Time.now() - self.startTime).to_sec() < self.timeout: 
            if self.dist_right == -1 or self.diff > self.diff_max:
                twist.angular.z = self.ang_vel
                rospy.loginfo("[align_table] Alinhando... Diferença: %.2f", self.diff)
            elif self.dist_left == -1 or self.diff < -self.diff_max:
                twist.angular.z = -self.ang_vel
                rospy.loginfo("[align_table] Alinhando... Diferença: %.2f", self.diff)
            elif abs(self.diff) <= self.diff_max:
                rospy.loginfo("[align_table] Robô alinhado")
                rospy.sleep(1)
                self.flag_align = False
                self.flag_finished = True
                if self.flag_start and not self.check_distance():
                    self.flag_start = False
                    if not self.just_align:
                        self.flag_fixDist = True
                        rospy.loginfo("[align_table] Ajustando distância com a mesa")
                else:
                    self.flag_finished = True

                self.pub_move.publish("parar,0.5")
        else:
            self.flag_finished = True
            self.flag_start = False

        self.pub_vel.publish(twist)

    def check_distance(self):
        dist = min(self.dist_left, self.dist_right)     # verificação baseada na menor distância  

        return (dist <= self.table_dist + self.tolerance and 
                dist >= self.table_dist - self.tolerance)

    def fix_distance(self):
        dist = min(self.dist_left, self.dist_right)     # ajuste baseado na menor distância
        twist = Twist()

        # if dist < self.table_dist + 15 + self.tolerance:
        #   self.pub_move.publish("tras,0.3")  
        # elif dist > self.table_dist + 15:
        #     twist.linear.x = self.lin_vel
        if dist > (self.table_dist + self.tolerance):
            if not self.trade == 3:
                self.pub_move.publish("parar,0.1")
                rospy.sleep(0.5)
            twist.linear.x = self.lin_vel
            self.trade = 3
        elif dist < (self.table_dist - self.tolerance):
            if not self.trade == 4:
                self.pub_move.publish("parar,0.1")
                rospy.sleep(0.5)
            twist.linear.x = -self.lin_vel
            self.trade = 4
        else:
            self.flag_fixDist = False
            rospy.loginfo("[align_table] Robô está próximo da mesa")
            self.pub_move.publish("parar,0.5")
            # self.flag_finished = True
            if abs(self.diff) > self.diff_max:
                self.flag_align = True
            else:
                self.flag_finished = True
        
        self.pub_vel.publish(twist)

    def spin(self):
        rate = rospy.Rate(50)  # 10 Hz
        while not rospy.is_shutdown():
            if self.flag_finished:
                self.publish_feedback()
                self.flag_align = False
                self.flag_fixDist = False
                self.flag_start = False
                self.flag_finished = False
            else:
                if self.flag_align:
                    self.align()
                elif self.flag_fixDist:
                    self.fix_distance()
            
            rate.sleep()


if __name__ == '__main__':
    try:
        align_table = MoveToTarget()
        align_table.spin()
    except rospy.ROSInterruptException:
        pass