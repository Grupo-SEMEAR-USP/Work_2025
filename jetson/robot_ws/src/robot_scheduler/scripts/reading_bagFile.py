#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from robot_scheduler.msg import TaskRequest
from collections import defaultdict
import json
import pprint

# Dicionário global
dados_local = defaultdict(list)

def callback(msg):
    if not msg.subtasks:
        rospy.loginfo("Mensagem recebida sem subtasks.")
        return

    rospy.loginfo("==== Nova mensagem recebida ====")
    rospy.loginfo(f"Quantidade de subtasks: {len(msg.subtasks)}")

    # Opcional: imprimir tudo
    print("Mensagem completa:")
    pprint.pprint(msg)

    for i, subtask in enumerate(msg.subtasks, 1):
        obj = subtask.object

        object_value = obj.object
        target_value = obj.target
        decoy_value = obj.decoy
        pose_value = obj.pose
        source_value = subtask.source
        destination_value = subtask.destination

        key = (source_value, destination_value)

        # Construir estrutura
        item = {
            "object": object_value,
            "target": target_value,
            "decoy": decoy_value,
            "pose": {
                "header": {
                    "frame_id": pose_value.header.frame_id,
                    "stamp": pose_value.header.stamp.to_sec()
                },
                "position": {
                    "x": pose_value.pose.position.x,
                    "y": pose_value.pose.position.y,
                    "z": pose_value.pose.position.z
                },
                "orientation": {
                    "x": pose_value.pose.orientation.x,
                    "y": pose_value.pose.orientation.y,
                    "z": pose_value.pose.orientation.z,
                    "w": pose_value.pose.orientation.w
                }
            }
        }

        # Salvar
        dados_local[key].append(item)

        # Log detalhado
        rospy.loginfo(f"Subtask #{i}:")
        rospy.loginfo(f"  Source: {source_value}")
        rospy.loginfo(f"  Destination: {destination_value}")
        rospy.loginfo(f"  Object ID: {object_value}")
        rospy.loginfo(f"  Target ID: {target_value}")
        rospy.loginfo(f"  Decoy: {decoy_value}")
        rospy.loginfo(f"  Pose: {item['pose']}")

def save_data():
    rospy.loginfo("Salvando dados no arquivo...")
    with open('dados_local.json', 'w') as f:
        json.dump(dados_local, f, indent=2)
    rospy.loginfo("Dados salvos com sucesso.")

def listener():
    rospy.init_node('data_saver', anonymous=True)
    rospy.Subscriber("/atwork_commander/object_task", TaskRequest, callback)
    rospy.on_shutdown(save_data)
    rospy.loginfo("Coletando dados... Pressione Ctrl+C para parar.")
    rospy.spin()

if __name__ == '__main__':
    listener()
