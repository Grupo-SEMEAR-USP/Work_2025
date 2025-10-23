#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int32
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from robot_base_controller.msg import UltrasonicDistances # <-- MUDANÇA: Importação correta

class ZigZagSearch:
    def __init__(self):
        rospy.init_node('search_block')

        # --- Publishers e Subscribers Corrigidos ---
        rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_cb)
        # <-- MUDANÇA: Corrigido o tipo da mensagem para a sua customizada
        rospy.Subscriber('/ultrasonic_distances', UltrasonicDistances, self.ultrasound_cb)
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_move_time = rospy.Publisher('/move_time', String, queue_size=10)
        self.pub_found_block = rospy.Publisher('/found_block', Int32, queue_size=10)
        self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)
        
        self.rate = rospy.Rate(10) # 10 Hz

        # --- Parâmetros ---
        self.distance_threshold = 0.15
        self.center_tolerance = 0.06
        self.lastID = None

        # --- Variáveis de Estado ---
        self.state = "IDLE"
        self.aruco_id_target = -1
        self.search_direction = "direita" # Valor padrão
        self.current_distance = float('inf')
        self.zigzag_step = 0
        self.move_end_time = rospy.Time.now() # Tempo em que o passo de movimento atual deve terminar

    def sched_cb(self, msg):
        if msg.target == "search_block" and self.state == "IDLE":
            rospy.loginfo("Comando de busca recebido!")
            self.lastID = msg.uid
            try:
                payload = msg.payload.split(',')
                self.aruco_id_target = int(payload[0])
                self.search_direction = payload[1]
                self.state = "FORWARD_ZIGZAG"
                self.zigzag_step = -1 # Começa em -1 para que o primeiro passo seja 0
                self.move_end_time = rospy.Time.now() # Inicia o primeiro passo imediatamente
            except (ValueError, IndexError) as e:
                rospy.logerr(f"Payload inválido: {msg.payload}. Erro: {e}")
                self.state = "IDLE"

    def tag_cb(self, data):
        if self.state != "FORWARD_ZIGZAG":
            return
            
        for detection in data.detections:
            if detection.id[0] == self.aruco_id_target:
                rospy.loginfo(f"Alvo {self.aruco_id_target} encontrado!")
                self.state = "FOUND"
                break
    
    def ultrasound_cb(self, msg):
        # <-- MUDANÇA: Agora acessa o campo nomeado da sua mensagem customizada
        self.current_distance = msg.rear_right

    def stop_all_movement(self):
        # Envia um comando de parada para o move_time
        self.pub_move_time.publish("parar,0")

    def run(self):
        while not rospy.is_shutdown():
            if self.state == "FORWARD_ZIGZAG":
                # --- LÓGICA NÃO BLOQUEANTE ---
                # Verifica se o tempo do movimento anterior já acabou
                if rospy.Time.now() >= self.move_end_time:
                    # Avança para o próximo passo
                    self.zigzag_step = (self.zigzag_step + 1) % 4
                    
                    # Define a sequência de movimentos aqui dentro para usar a direção correta
                    zigzag_sequence = [
                        (self.search_direction, 2.0),
                        ("tras", 1.1),
                        (self.search_direction, 2.0),
                        ("frente", 1.1)
                    ]
                    
                    direction, duration = zigzag_sequence[self.zigzag_step]
                    
                    # Publica o comando para o próximo passo APENAS UMA VEZ
                    self.pub_move_time.publish(f"{direction},{duration}")
                    rospy.loginfo(f"Iniciando passo {self.zigzag_step}: movendo para '{direction}' por {duration}s")
                    
                    # Define quando este passo deve terminar
                    self.move_end_time = rospy.Time.now() + rospy.Duration(duration)

                # Condição de parada por distância (verificada continuamente)
                if self.current_distance < self.distance_threshold:
                    rospy.loginfo("Parede detectada! Zigue-zague interrompido.")
                    self.stop_all_movement()
                    self.state = "IDLE" 

            elif self.state == "FOUND":
                self.stop_all_movement()
                rospy.loginfo("Alvo encontrado! Tarefa concluída.")
                
                feedback = SchedulerResponse(uid=self.lastID, target="search_block", status=True)
                self.pub_feedback.publish(feedback)
                
                self.pub_found_block.publish(self.aruco_id_target)
                self.state = "IDLE"

            self.rate.sleep()

if __name__ == '__main__':
    try:
        search_node = ZigZagSearch()
        search_node.run()
    except rospy.ROSInterruptException:
        pass