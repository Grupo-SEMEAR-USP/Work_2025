#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Int32

# --- Variáveis Globais para os Publishers ---
# Crie os publishers aqui, apenas UMA VEZ, para eficiência.
pub_cmd_vel = None
pub_feedback = None

def move_callback(msg):
    """
    Callback que recebe um comando de movimento, o executa por uma
    duração específica e publica um feedback ao concluir.
    Agora é robusto contra mensagens malformadas.
    """
    global pub_cmd_vel, pub_feedback # Informa que vamos usar as variáveis globais

    try:
        # --- MELHORIA 1: Validar o formato da mensagem ---
        parts = msg.data.split(',')
        if len(parts) != 2:
            rospy.logwarn(f"Mensagem inválida recebida: formato incorreto. Esperado 'direcao,duracao', recebido '{msg.data}'")
            return

        direction = parts[0].strip() # .strip() remove espaços em branco acidentais
        duration_str = parts[1].strip()

        # --- MELHORIA 2: Validar se a duração é um número válido ---
        try:
            duration = float(duration_str)
        except ValueError:
            rospy.logwarn(f"Mensagem inválida recebida: a duração '{duration_str}' não é um número válido.")
            return

        # A sua lógica de movimento (não precisa de mudanças)
        move_cmd = Twist()

        if direction == 'frente':
            move_cmd.linear.x = 0.2
        elif direction == "parar":
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
        elif direction == 'tras':
            move_cmd.linear.x = -0.2
        elif direction == 'esquerda':
            move_cmd.angular.z = 0.6
        elif direction == 'direita':
            move_cmd.angular.z = -0.6
        elif direction == 'direita_90':
            move_cmd.linear.y = -0.6
        elif direction == 'esquerda_90':
            move_cmd.linear.y = 0.6
        else:
            rospy.logwarn(f"Direção não reconhecida: '{direction}'")
            return

        rate = rospy.Rate(50)
        start_time = rospy.Time.now()
        duration_time = rospy.Duration(duration)

        # Loop de movimento
        while rospy.Time.now() - start_time < duration_time and not rospy.is_shutdown():
            pub_cmd_vel.publish(move_cmd)
            rate.sleep()
        
        # Garante que o robô pare após o tempo
        stop_cmd = Twist()
        pub_cmd_vel.publish(stop_cmd)

        rospy.loginfo(f"Movimento completado: '{direction}' por {duration} segundos.")
        
        pub_feedback.publish(1)

    except Exception as e:
        # Este 'except' genérico ainda é útil para pegar outros erros inesperados
        rospy.logerr(f"Erro inesperado ao processar a mensagem: {str(e)}")

def listener():
    global pub_cmd_vel, pub_feedback
    
    rospy.init_node('move_time_listener', anonymous=True)

    # Inicializa os Publishers aqui, na função principal
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_feedback = rospy.Publisher('/move_time/feedback', Int32, queue_size=10)

    rospy.Subscriber("move_time", String, move_callback)
    
    rospy.loginfo("Nó 'move_time' iniciado e aguardando comandos.")
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass