# #!/usr/bin/env python3

# import rospy
# import numpy as np
# from std_msgs.msg import String, Int32, Float32MultiArray
# from geometry_msgs.msg import Twist  # Para controlar o movimento do robô
# from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
# from collections import deque

# TARGET_NAME = "border_detect"
# BUFFER_SIZE = 5  # Número de leituras para média móvel

# class TableEdgeDetector:
#     def __init__(self):
#         rospy.init_node('table_edge_detector', anonymous=True)
        
#         # Subscritor para iniciar a detecção ao receber uma mensagem
#         self.edge_detect_sub = rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)

#         # Subscritor de imagem de profundidade
#         self.depth_sub = rospy.Subscriber("/ultrasonic_distances", Float32MultiArray, self.ultrasonic_edge_callback, queue_size=10)

#         # Publisher de movimento do robô
#         self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

#         # Publisher de feedback para o scheduler
#         self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)
        
#         # Variáveis de controle
#         self.twist = Twist()
#         self.edge_detected = False
#         self.movement_active = False
#         self.search_direction = None

#         self.lin_vel = 0.1
#         self.edge_threshold_cm = 15.0 

#         self.buffer_left = deque(maxlen=BUFFER_SIZE)
#         self.buffer_right = deque(maxlen=BUFFER_SIZE)
        
#         self.lastID = None

#         rospy.loginfo("TableEdgeDetector iniciado, aguardando comando de detecção...")


#     def sched_cb(self, msg):
#         if msg.target != TARGET_NAME or msg.uid == self.lastID:
#             return
#         rospy.loginfo(f"[border_detect] Comando para iniciar a busca por borda recebido!")
#         self.lastID = msg.uid
#         self.search_direction = msg.payload
#         self.movement_active = True
#         self.edge_detected = False
#         self.flag_finished = False

#     def publish_feedback(self):
#         feedback = SchedulerResponse()
#         feedback.uid = self.lastID
#         feedback.target = TARGET_NAME
#         feedback.status = True
#         for _ in range(5):
#             self.pub_feedback.publish(feedback)

#     def ultrasonic_edge_callback(self, msg):
#         if not self.movement_active or self.edge_detected:
#             return

#         # Validação para garantir que o array tem pelo menos os 2 sensores que precisamos
#         if len(msg.data) < 2:
#             rospy.logwarn_throttle(5, "[border_detect] Array de ultrassom recebido é muito curto!")
#             return

#         # Adiciona as novas leituras aos buffers usando o índice correto
#         front_left_reading = msg.data[0]
#         front_right_reading = msg.data[1]
        
#         self.buffer_left.append(front_left_reading)
#         self.buffer_right.append(front_right_reading)

#         # Procede apenas quando os buffers estiverem cheios
#         if len(self.buffer_left) == BUFFER_SIZE:
#             avg_left = sum(self.buffer_left) / BUFFER_SIZE
#             avg_right = sum(self.buffer_right) / BUFFER_SIZE
            
#             diff = abs(avg_left - avg_right)

#             rospy.loginfo(f"[border_detect] Médias: Esq={avg_left:.2f} cm, Dir={avg_right:.2f} cm, Diff={diff:.2f} cm")

#             # Verifica se a diferença excedeu o limiar de detecção
#             if diff > self.edge_threshold_cm:
#                 rospy.loginfo(f"BORDA DETECTADA! Diferença ({diff:.2f} cm) excedeu o limiar ({self.edge_threshold_cm} cm).")
#                 self.edge_detected = True

#     def move_robot(self):
#         self.twist.linear.x = 0.0
#         if self.search_direction == "direita":
#             self.twist.linear.y = -self.lin_vel  # Deslocamento para a direita
#             rospy.loginfo("Movendo para a direita...")
#         elif self.search_direction == "esquerda":
#             self.twist.linear.y = self.lin_vel  # Deslocamento para a esquerda
#             rospy.loginfo("Movendo para a esquerda...")
#         self.cmd_vel_pub.publish(self.twist)

#     def stop_robot(self):
#         self.movement_active = False
#         self.twist.linear.x = 0.0
#         self.twist.linear.y = 0.0
#         self.cmd_vel_pub.publish(self.twist)
#         rospy.loginfo("Robô parado.")

#     def move(self):
#         rate = rospy.Rate(10)  # 10 Hz
#         while not rospy.is_shutdown():
#             if self.movement_active:
#                 if not self.edge_detected:
#                     self.move_robot()
#                 else:
#                     self.stop_robot()
#             elif self.edge_detected:
#                 self.stop_robot()
#                 self.publish_feedback()
#                 self.edge_detected = False
#                 self.movement_active = False
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         detector = TableEdgeDetector()
#         detector.move()
#     except rospy.ROSInterruptException:
#         pass

#SUPOSTA CORREÇÃO

# #!/usr/bin/env python3
# import rospy
# from geometry_msgs.msg import Twist
# from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
# from robot_base_controller.msg import UltrasonicDistances
# from collections import deque

# TARGET_NAME = "border_detect"
# BUFFER_SIZE = 5  # Número de leituras para média móvel

# class TableEdgeDetector:
#     def __init__(self):
#         rospy.init_node('border_detect', anonymous=True)
        
#         # Subscritor do comando do scheduler
#         self.edge_detect_sub = rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)

#         # Subscritor das leituras ultrassônicas (mensagem estruturada)
#         self.depth_sub = rospy.Subscriber("/ultrasonic_distances", UltrasonicDistances, self.ultrasonic_edge_callback, queue_size=10)

#         # Publicadores
#         self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
#         self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)
        
#         # Variáveis internas
#         self.twist = Twist()
#         self.edge_detected = False
#         self.movement_active = False
#         self.search_direction = None

#         # Configurações de movimento
#         self.lin_vel = 0.1
#         self.edge_threshold_cm = 15.0  # diferença entre sensores (em cm)

#         # Buffers para médias móveis
#         self.buffer_left = deque(maxlen=BUFFER_SIZE)
#         self.buffer_right = deque(maxlen=BUFFER_SIZE)
        
#         self.lastID = None

#         rospy.loginfo("[border_detect] Nó iniciado. Aguardando comando...")

#     def sched_cb(self, msg):
#         """Callback que ativa a detecção ao receber um comando válido do scheduler."""
#         if msg.target != TARGET_NAME or msg.uid == self.lastID:
#             return

#         rospy.loginfo(f"[border_detect] Comando recebido! Iniciando detecção de borda.")
#         self.lastID = msg.uid
#         self.search_direction = msg.payload
#         self.movement_active = True
#         self.edge_detected = False
#         self.flag_finished = False

#     def publish_feedback(self):
#         """Envia resposta positiva para o scheduler quando a borda é detectada."""
#         feedback = SchedulerResponse()
#         feedback.uid = self.lastID
#         feedback.target = TARGET_NAME
#         feedback.status = True
#         for _ in range(5):
#             self.pub_feedback.publish(feedback)
#         rospy.loginfo("[border_detect] Feedback enviado ao scheduler.")

#     def ultrasonic_edge_callback(self, msg: UltrasonicDistances):
#         """Recebe as leituras ultrassônicas estruturadas e detecta diferença de altura (borda)."""
#         if not self.movement_active or self.edge_detected:
#             return

#         # Converte para centímetros (caso os sensores reportem em metros)
#         front_left = msg.front_left * 100.0
#         front_right = msg.front_right * 100.0

#         # Adiciona as leituras aos buffers
#         self.buffer_left.append(front_left)
#         self.buffer_right.append(front_right)

#         # Quando o buffer estiver completo, calcula as médias
#         if len(self.buffer_left) == BUFFER_SIZE:
#             avg_left = sum(self.buffer_left) / BUFFER_SIZE
#             avg_right = sum(self.buffer_right) / BUFFER_SIZE
#             diff = abs(avg_left - avg_right)

#             rospy.loginfo(f"[border_detect] Médias: Esq={avg_left:.1f} cm, Dir={avg_right:.1f} cm, Diferença={diff:.1f} cm")

#             # Detecta borda
#             if diff > self.edge_threshold_cm:
#                 rospy.logwarn(f"[border_detect] BORDA DETECTADA! Diferença {diff:.1f} cm > limiar {self.edge_threshold_cm:.1f} cm")
#                 self.edge_detected = True

#     def move_robot(self):
#         """Comanda o robô a mover-se lateralmente enquanto busca a borda."""
#         self.twist.linear.x = 0.0
#         if self.search_direction == "direita":
#             self.twist.linear.y = -self.lin_vel
#             rospy.loginfo("[border_detect] Movendo para a direita...")
#         elif self.search_direction == "esquerda":
#             self.twist.linear.y = self.lin_vel
#             rospy.loginfo("[border_detect] Movendo para a esquerda...")
#         self.cmd_vel_pub.publish(self.twist)

#     def stop_robot(self):
#         """Para completamente o robô."""
#         self.movement_active = False
#         self.twist.linear.x = 0.0
#         self.twist.linear.y = 0.0
#         self.cmd_vel_pub.publish(self.twist)
#         rospy.loginfo("[border_detect] Robô parado.")

#     def move(self):
#         """Loop principal de movimentação e detecção."""
#         rate = rospy.Rate(10)
#         while not rospy.is_shutdown():
#             if self.movement_active:
#                 if not self.edge_detected:
#                     self.move_robot()
#                 else:
#                     self.stop_robot()
#             elif self.edge_detected:
#                 self.stop_robot()
#                 self.publish_feedback()
#                 self.edge_detected = False
#                 self.movement_active = False
#             rate.sleep()

# if __name__ == '__main__':
#     try:
#         detector = TableEdgeDetector()
#         detector.move()
#     except rospy.ROSInterruptException:
#         pass

#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from robot_base_controller.msg import UltrasonicDistances
from collections import deque

TARGET_NAME = "border_detect"
BUFFER_SIZE = 5  # Número de leituras para média móvel

class TableEdgeDetector:
    def __init__(self):
        rospy.init_node('border_detect', anonymous=True)
        
        # Subscribers
        self.edge_detect_sub = rospy.Subscriber("/scheduler/commands", SchedulerCommand, self.sched_cb, queue_size=10)
        self.depth_sub = rospy.Subscriber("/ultrasonic_distances", UltrasonicDistances, self.ultrasonic_edge_callback, queue_size=10)

        # Publishers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.pub_feedback = rospy.Publisher("/scheduler/feedback", SchedulerResponse, queue_size=10)
        
        # Variáveis internas
        self.twist = Twist()
        self.edge_detected = False
        self.movement_active = False
        self.search_direction = None
        self.timeout = None
        self.start_time = None

        # Configurações de movimento
        self.lin_vel = 0.05
        self.edge_threshold_cm = 15.0  # diferença entre sensores (em cm)

        # Buffers para médias móveis
        self.buffer_left = deque(maxlen=BUFFER_SIZE)
        self.buffer_right = deque(maxlen=BUFFER_SIZE)
        
        self.lastID = None

        rospy.loginfo("[border_detect] Nó iniciado. Aguardando comando...")

    # CALLBACK DO SCHEDULER

    def sched_cb(self, msg):
        """Callback que ativa a detecção ao receber um comando válido do scheduler."""
        if msg.target != TARGET_NAME or msg.uid == self.lastID:
            return

        self.lastID = msg.uid
        self.search_direction = msg.payload
        self.movement_active = True
        self.edge_detected = False
        self.timeout = msg.timeout  # timeout padrão se não vier no comando
        self.start_time = rospy.Time.now()

        rospy.loginfo(f"[border_detect] Comando recebido! Direção: {self.search_direction}, timeout={self.timeout}s")

    # PUBLICAÇÃO DE FEEDBACK

    def publish_feedback(self, success=True):
        """Envia resposta (OK ou FAIL) para o scheduler."""
        feedback = SchedulerResponse()
        feedback.uid = self.lastID
        feedback.target = TARGET_NAME
        status_str = "OK" if success else "FAIL"
        feedback.status = status_str

        
        rospy.loginfo(f"[border_detect] Feedback: {status_str}")

        for _ in range(5):
            self.pub_feedback.publish(feedback)
            rospy.sleep(0.1)

    # CALLBACK ULTRASSÔNICO

    def ultrasonic_edge_callback(self, msg: UltrasonicDistances):
        """Recebe as leituras ultrassônicas estruturadas e detecta diferença de altura (borda)."""
        if not self.movement_active or self.edge_detected:
            return

        # Converte para centímetros
        front_left = msg.front_left
        front_right = msg.front_right

        # Adiciona as leituras aos buffers
        self.buffer_left.append(front_left)
        self.buffer_right.append(front_right)

        # Quando o buffer estiver completo, calcula as médias
        if len(self.buffer_left) == BUFFER_SIZE:
            avg_left = sum(self.buffer_left) / BUFFER_SIZE
            avg_right = sum(self.buffer_right) / BUFFER_SIZE
            diff = abs(avg_left - avg_right)

            rospy.loginfo(f"[border_detect] Médias: Esq={avg_left:.1f} cm, Dir={avg_right:.1f} cm, Diferença={diff:.1f} cm")

            # Detecta borda
            if diff > self.edge_threshold_cm:
                rospy.logwarn(f"[border_detect] BORDA DETECTADA! Diferença {diff:.1f} cm > limiar {self.edge_threshold_cm:.1f} cm")
                self.edge_detected = True

    # MOVIMENTO

    def move_robot(self):
        """Comanda o robô a mover-se lateralmente enquanto busca a borda."""
        self.twist.linear.x = 0.0
        if self.search_direction == "direita":
            self.twist.linear.y = -self.lin_vel
            rospy.loginfo_throttle(2, "[border_detect] Movendo para a direita...")
        elif self.search_direction == "esquerda":
            self.twist.linear.y = self.lin_vel
            rospy.loginfo_throttle(2, "[border_detect] Movendo para a esquerda...")
        self.cmd_vel_pub.publish(self.twist)

    def stop_robot(self):
        """Para completamente o robô."""
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("[border_detect] Robô parado.")

    # LOOP PRINCIPAL

    def move(self):
        """Loop principal de movimentação e detecção."""
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.movement_active:
                elapsed = (rospy.Time.now() - self.start_time).to_sec()

                # Se detectou borda -> sucesso
                if self.edge_detected:
                    self.stop_robot()
                    self.publish_feedback(success=True)
                    self.movement_active = False
                    continue

                # Se atingiu o timeout -> falha
                if elapsed > self.timeout:
                    rospy.logwarn(f"[border_detect] Timeout ({self.timeout}s) atingido sem detectar borda.")
                    self.stop_robot()
                    self.publish_feedback(success=False)
                    self.movement_active = False
                    continue

                # Continua procurando
                self.move_robot()

            rate.sleep()

if __name__ == '__main__':
    try:
        detector = TableEdgeDetector()
        detector.move()
    except rospy.ROSInterruptException:
        pass
