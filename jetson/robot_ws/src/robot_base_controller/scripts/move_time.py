#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from robot_scheduler.msg import SchedulerCommand
from std_msgs.msg import String, Int32

def move_callback(msg):
    try:
        direction, duration = msg.data.split(',')
        duration = float(duration)

        move_cmd = Twist()

        lin_vel_x = 0.2    
        lin_vel_y = 0.2
        ang_vel_z = 0.6

        if direction == 'frente':
            move_cmd.linear.x = lin_vel_x
        elif direction == "parar":
            move_cmd.linear.x = 0
            move_cmd.linear.y = 0
            move_cmd.angular.z = 0
        elif direction == 'tras':
            move_cmd.linear.x = -lin_vel_x
        elif direction == 'esquerda':
            move_cmd.linear.y = lin_vel_y
        elif direction == 'direita':
            move_cmd.linear.y = -lin_vel_y
        elif direction ==  'direita_90':
            move_cmd.angular.z = -ang_vel_z
        elif direction == 'esquerda_90':
            move_cmd.angular.z = ang_vel_z
        else:
            rospy.logwarn("Direção não reconhecida. Use 'frente', 'tras', 'esquerda' ou 'direita'.")
            return

        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        feedback_pub = rospy.Publisher('/move_time/feedback', Int32, queue_size=10)
        rate = rospy.Rate(50)  # 50 Hz
        
        start_time = rospy.Time.now()
        duration_time = rospy.Duration(duration)

        while rospy.Time.now() - start_time < duration_time:
            pub.publish(move_cmd)
            rate.sleep()
        
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.angular.z = 0.0
        pub.publish(move_cmd)

        # rospy.loginfo(f"Movimento completado: {direction} por {duration} segundos.")
        
        feedback_pub.publish(1)

    except Exception as e:
        rospy.logerr(f"Erro ao processar a mensagem: {str(e)}")

def sched_cb(msg):
    global name, last_id

    if msg.target != name:
        return

    if msg.uid == last_id:
        rospy.logdebug(f"ID {msg.uid} já recebido - ignorando.")
        return

    last_id = msg.uid

    msg_ = String(msg.payload)
    time = msg.timeout

    real_msg = String(msg_.data+","+str(time)) 
    rospy.loginfo(real_msg.data)
    move_callback(real_msg)

def listener():
    rospy.init_node('move_time_listener', anonymous=True)

    rospy.Subscriber("move_time", String, move_callback)
    rospy.Subscriber("scheduler/commands", SchedulerCommand, sched_cb)

    rospy.spin()

if __name__ == '__main__':
    try:
        global name, last_id
        last_id = None
        name = "move"
        listener()
    except rospy.ROSInterruptException:
        pass
