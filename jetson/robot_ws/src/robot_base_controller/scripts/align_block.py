#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse 

class AlignBlock:
    def __init__(self):

        rospy.init_node('align_block', anonymous=True)
        self.node_name = "align_block"
        self.last_id = None
        self.last_time = None
        self.max_time = 3   # maximo tempo pra procurar o bloco após perder e estabilizar
        self.search_active = False
        self.has_converged = False
        self.timeout = None
        self.init_time = None

        self.target_block_id = None
        self.incorrect_detection_count = 0  # Contador de leituras incorretas
        self.last_valid_position = None     # Armazena a última posição válida do bloco

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.feedback_pub = rospy.Publisher('/scheduler/feedback', SchedulerResponse, queue_size=10)

        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detections_callback)
        rospy.Subscriber('/scheduler/commands', SchedulerCommand, self.sched_callback)

        rospy.loginfo("[align_block] Inicializado e aguardando comandos...")


    def sched_callback(self, msg: SchedulerCommand):
        if msg.target != self.node_name:
                return

        if msg.uid == self.last_id:
            rospy.logdebug(f"ID {msg.uid} já recebido - ignorando.")
            return

        self.timeout = msg.timeout
        self.target_block_id = int(msg.payload)
        self.last_id = msg.uid
        self.incorrect_detection_count = 0  # Reseta o contador ao iniciar uma nova busca
        self.last_valid_position = None     # Reseta a última posição válida
        self.search_active = True
        self.init_time = rospy.Time.now()

        rospy.loginfo(f"Busca pelo bloco de ID {self.target_block_id} iniciado.")
        
        
    def tag_detections_callback(self, data: AprilTagDetectionArray):
        
        if not self.search_active:
            # rospy.loginfo("ignorando camera - serach active false")
            return

        if self.has_converged:
            # rospy.loginfo("ignorando camera - has converged")
            return

        # Timeout
        # rospy.loginfo("a")
        if((rospy.Time.now() - self.init_time).to_sec() >= self.timeout):
            rospy.loginfo(f"Timeout da procura de blocos.")
            self.stop_robot()
                                
        detected_target = False

        # Verifica as tags detectadas
        for detection in data.detections:
            tag_id = detection.id[0]
            if tag_id == self.target_block_id:
                detected_target = True  # Alvo detectado
                rospy.loginfo("ACHEI")
                self.last_valid_position = detection.pose.pose.pose.position  # Atualiza a última posição válida

        # Se o bloco alvo não foi encontrado, incrementa o contador de leituras incorretas
        if not detected_target:
            rospy.loginfo(f"Bloco alvo não detectado.")

            # Para o robô
            cmd_vel = Twist()
            cmd_vel.linear.x = 0
            cmd_vel.linear.y = 0
            self.vel_pub.publish(cmd_vel)
            
            if self.last_time == None:
                self.last_time = rospy.Time.now()

            else:
                duration =  rospy.Time.now() - self.last_time
                # rospy.loginfo(f'duration = {duration.to_sec()}')
                if duration.to_sec() > self.max_time:
                    self.search_active = False
                    rospy.loginfo("Bloco alvo não encontrado após. Desistindo.")
                    self.stop_robot()

                    fb_msg = SchedulerResponse(self.last_id, self.node_name, "FAIL")
                    for _ in range(5):
                        self.feedback_pub.publish(fb_msg)
                        rospy.sleep(0.1)
            return
    
        else:
            self.last_time = None

            # Reseta o contador se o bloco alvo for encontrado
            if self.last_valid_position:  # Tenta alinhar-se usando a posição atualizada
                self.align_with_position(self.last_valid_position)


    def align_with_position(self, position):
        cmd_vel = Twist()

        # Define o deslocamento da garra em relação ao centro da câmera
        # ta certo esse offset
        offset_y = 0.0475    # 47,5 mm
        offset_x = 0.0

        # Compensa o deslocamento da garra ao calcular as coordenadas de alinhamento
        adjusted_x = position.x - offset_x
        adjusted_y = position.y - offset_y

        # Prioridade para o alinhamento horizontal (eixo Y)
        tol_x = 0.01
        tol_y = 0.01
        vel_x = 0.1
        vel_y = 0.3

        if abs(adjusted_y) > tol_y: 
            rospy.loginfo("Ajustando X")

            cmd_vel.linear.x = -vel_x if adjusted_y > tol_y else vel_x  # Ajusta para alinhar no eixo Y
            cmd_vel.angular.y = 0  # Não move no eixo X até que Y esteja alinhado
            rospy.loginfo(f"vel = {cmd_vel.linear.x}")
            

        # Alinhamento vertical (eixo X) somente quando o eixo Y estiver alinhado
        elif abs(adjusted_x) > tol_x:  
            rospy.loginfo("Ajustando Y")
            cmd_vel.linear.x = 0  # Para a movimentação no eixo Y
            cmd_vel.angular.y = -vel_y if adjusted_x > tol_x else vel_y  # Ajusta para alinhar no eixo X
            rospy.loginfo(f"vel = {cmd_vel.linear.y}")

        # Se o robô está alinhado em ambos os eixos
        else:
            cmd_vel.linear.x = 0
            cmd_vel.angular.y = 0
            if not self.has_converged:
                rospy.loginfo("Robô alinhado com o bloco.")
                self.has_converged = True  # Marca o robô como alinhado
                self.search_active = False
                self.target_block_id = None
                
                fb_msg = SchedulerResponse(self.last_id, self.node_name, "OK")
                for _ in range(5):
                    self.feedback_pub.publish(fb_msg)
                    rospy.sleep(0.1)

        # Publica os comandos de movimento
        # for _ in range(2):
        self.vel_pub.publish(cmd_vel)
            # rospy.sleep(0.1)

        # Publica os comandos de movimento
        # time = rospy.Time.now()
        # while True:
        #     duration =  rospy.Time.now() - time
        #     rospy.loginfo(f"duration: {duration.to_sec()}")
        #     if duration.to_sec() > 0.5:
        #         break
        #     self.vel_pub.publish(cmd_vel)
            
        # rospy.sleep(0.5)

    def stop_robot(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0
        cmd_vel.angular.y = 0
        self.vel_pub.publish(cmd_vel)
        # rospy.loginfo("stop called")
        self.search_active = False


if __name__ == '__main__':
    try:
        AlignBlock()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass