#!/usr/bin/env python3

import rospy
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, String

has_converged = False
search_active = False
alignment_pub = None
target_block_id = None

incorrect_detection_count = 0 
max_incorrect_detections = 8  
last_valid_position = None    

def start_search_callback(msg):
    global search_active, has_converged, target_block_id, incorrect_detection_count, last_valid_position
    search_active = True
    has_converged = False
    target_block_id = int(msg.data)
    incorrect_detection_count = 0 
    last_valid_position = None 
    rospy.loginfo(f"Buscando pelo bloco de ID {target_block_id} iniciado.")

def tag_detections_callback(data):
    global has_converged, search_active, alignment_pub, target_block_id, incorrect_detection_count, last_valid_position

    if not search_active:
        return

    if has_converged:
        return

    detected_target = False

    for detection in data.detections:
        tag_id = detection.id[0]
        if tag_id == target_block_id:
            detected_target = True 
            last_valid_position = detection.pose.pose.pose.position 

    if not detected_target:
        incorrect_detection_count += 1
        rospy.loginfo(f"Bloco alvo não detectado. Leituras incorretas: {incorrect_detection_count}")
        
        if incorrect_detection_count >= max_incorrect_detections:
            rospy.loginfo("Bloco alvo não encontrado após múltiplas tentativas. Desistindo.")
            alignment_pub.publish(0)
            stop_robot()
            search_active = False
        elif last_valid_position: 
            align_with_position(last_valid_position)
        return
    else:
        incorrect_detection_count = 0
        if last_valid_position:  
            align_with_position(last_valid_position)

def align_with_position(position):
    global has_converged, alignment_pub, search_active
    cmd_vel = Twist()

    offset_x = 0.0050260697861753125
    offset_y = 0.03122536780988897

    adjusted_x = position.x - offset_x
    adjusted_y = position.y - offset_y

    # Prioridade para o alinhamento horizontal (eixo Y)
    if abs(adjusted_y) > 0.01:  
        cmd_vel.linear.x = -0.08 if adjusted_y > 0.01 else 0.08 
        cmd_vel.linear.y = 0 

    # Alinhamento vertical (eixo X) somente quando o eixo Y estiver alinhado
    elif abs(adjusted_x) > 0.005:  
        cmd_vel.linear.x = 0 
        cmd_vel.angular.z = -0.08 if adjusted_x > 0.005 else 0.08 

    else:
        cmd_vel.linear.x = 0
        cmd_vel.linear.y = 0
        if not has_converged:
            rospy.loginfo("Robô alinhado com o bloco.")
            send_found_signal()
            
            for b in range(5):
                alignment_pub.publish(1)
                rospy.sleep(0.1)

            has_converged = True
            search_active = False
            target_block_id = None

    vel_pub.publish(cmd_vel)

def send_found_signal():
    global search_active
    rospy.loginfo("Bloco alinhado!")
    search_active = False

def stop_robot():
    cmd_vel = Twist()
    cmd_vel.linear.x = 0
    cmd_vel.linear.y = 0
    vel_pub.publish(cmd_vel)
    search_active = False

def main():
    global vel_pub, alignment_pub

    rospy.init_node('align_block', anonymous=True)

    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    alignment_pub = rospy.Publisher('/block_align/feedback', Int32, queue_size=10)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, tag_detections_callback)
    rospy.Subscriber('/block_align', String, start_search_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass