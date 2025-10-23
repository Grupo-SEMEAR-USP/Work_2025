#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import String, Int32, Float32MultiArray
from geometry_msgs.msg import Twist  # Para controlar o movimento do robô
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from collections import deque

TARGET_NAME = "border_detect"
BUFFER_SIZE = 5  # Número de leituras para média móvel

class TableEdgeDetector:
    def __init__(self):
        rospy.init_node('table_edge_detector', anonymous=True)
        
        # Subscritor para iniciar a detecção ao receber uma mensagem
        self.edge_detect_sub = rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)

        # Subscritor de imagem de profundidade
        self.depth_sub = rospy.Subscriber("/ultrasonic_distances", Float32MultiArray, self.ultrasonic_edge_callback, queue_size=10)

        # Publisher de movimento do robô
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Publisher de feedback para o scheduler
        self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)
        
        # Variáveis de controle
        self.twist = Twist()
        self.edge_detected = False
        self.movement_active = False
        self.search_direction = None

        self.lin_vel = 0.1
        self.edge_threshold_cm = 15.0 

        self.buffer_left = deque(maxlen=BUFFER_SIZE)
        self.buffer_right = deque(maxlen=BUFFER_SIZE)
        
        self.lastID = None

        rospy.loginfo("TableEdgeDetector iniciado, aguardando comando de detecção...")


    def sched_cb(self, msg):
        if msg.target != TARGET_NAME or msg.uid == self.lastID:
            return
        rospy.loginfo(f"[border_detect] Comando para iniciar a busca por borda recebido!")
        self.lastID = msg.uid
        self.search_direction = msg.payload
        self.movement_active = True
        self.edge_detected = False
        self.flag_finished = False

    def publish_feedback(self):
        feedback = SchedulerResponse()
        feedback.uid = self.lastID
        feedback.target = TARGET_NAME
        feedback.status = True
        for _ in range(5):
            self.pub_feedback.publish(feedback)

    def ultrasonic_edge_callback(self, msg):
        if not self.movement_active or self.edge_detected:
            return

        # Validação para garantir que o array tem pelo menos os 2 sensores que precisamos
        if len(msg.data) < 2:
            rospy.logwarn_throttle(5, "[border_detect] Array de ultrassom recebido é muito curto!")
            return

        # Adiciona as novas leituras aos buffers usando o índice correto
        front_left_reading = msg.data[0]
        front_right_reading = msg.data[1]
        
        self.buffer_left.append(front_left_reading)
        self.buffer_right.append(front_right_reading)

        # Procede apenas quando os buffers estiverem cheios
        if len(self.buffer_left) == BUFFER_SIZE:
            avg_left = sum(self.buffer_left) / BUFFER_SIZE
            avg_right = sum(self.buffer_right) / BUFFER_SIZE
            
            diff = abs(avg_left - avg_right)

            rospy.loginfo(f"[border_detect] Médias: Esq={avg_left:.2f} cm, Dir={avg_right:.2f} cm, Diff={diff:.2f} cm")

            # Verifica se a diferença excedeu o limiar de detecção
            if diff > self.edge_threshold_cm:
                rospy.loginfo(f"BORDA DETECTADA! Diferença ({diff:.2f} cm) excedeu o limiar ({self.edge_threshold_cm} cm).")
                self.edge_detected = True

    def move_robot(self):
        self.twist.linear.x = 0.0
        if self.search_direction == "direita":
            self.twist.linear.y = -self.lin_vel  # Deslocamento para a direita
            rospy.loginfo("Movendo para a direita...")
        elif self.search_direction == "esquerda":
            self.twist.linear.y = self.lin_vel  # Deslocamento para a esquerda
            rospy.loginfo("Movendo para a esquerda...")
        self.cmd_vel_pub.publish(self.twist)

    def stop_robot(self):
        self.movement_active = False
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Robô parado.")

    def move(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.movement_active:
                if not self.edge_detected:
                    self.move_robot()
                else:
                    self.stop_robot()
            elif self.edge_detected:
                self.stop_robot()
                self.publish_feedback()
                self.edge_detected = False
                self.movement_active = False
            rate.sleep()

if __name__ == '__main__':
    try:
        detector = TableEdgeDetector()
        detector.move()
    except rospy.ROSInterruptException:
        pass

