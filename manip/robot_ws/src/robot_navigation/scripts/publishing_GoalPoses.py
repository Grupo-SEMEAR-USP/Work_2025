#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Lista de waypoints: cada elemento é uma tupla ((x, y, z), (qx, qy, qz, qw))
# onde (x, y, z) define a posição e (qx, qy, qz, qw) define a orientação (quaternion).
waypoints = [
    ((-1.977, -1.021, 0.0), (0.0, 0.0, 0.7215208310173468, 0.6923927284482683)), 
    ((-1.916, 1.927, 0.0), (0.0, 0.0,-0.00397220280374852, 0.999992110771323)), 
    ((1.06, 0.954, 0.0), (0.0, 0.0, -0.7217794186514265, 0.692123161591352)), 
    ((1.80, -1.79, 0.0), (0.0, 0.0, -0.9933720755682208, 0.11494311410991477)), 
    ((-0.478, -0.916, 0.0), (0.0, 0.0, -0.997827524800359, 0.06588042767612359)),
    ((1.932, 1.303, 0.0), (0.0, 0.0, 0.6088056800085474, 0.7933193833440162))

]

def goal_pose(pose):
    """
    Converte a tupla 'pose' em um objeto MoveBaseGoal,
    definindo posição e orientação no frame 'map'.
    """
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'

    # Pose[0] é a posição (x, y, z)
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]

    # Pose[1] é a orientação (qx, qy, qz, qw)
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]

    return goal_pose

if __name__ == '__main__':
    rospy.init_node('patrol')

    # Cria um cliente de ação para o servidor 'move_base'
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    # Envia cada waypoint como um objetivo para o move_base e aguarda o resultado
    for pose in waypoints:
        goal = goal_pose(pose)
        client.send_goal(goal)
        client.wait_for_result()
