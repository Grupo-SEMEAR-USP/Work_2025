#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState
from robot_base_controller.msg import encoder_data, velocity_data
from paho.mqtt import client as mqtt
import threading

BROKER = "192.168.0.101"
PORT = 1883

TOPIC_ENCODER_FRONT = "estado/encoders/front"
TOPIC_ENCODER_REAR = "estado/encoders/rear"
TOPIC_COMMAND_FRONT = "comandos/motores/front"
TOPIC_COMMAND_REAR = "comandos/motores/rear"

encoder_msg = encoder_data()
lock = threading.Lock()

class MQTTBridge:
    def __init__(self):
        self.client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.pub_encoder = rospy.Publisher('/encoder_data', encoder_data, queue_size=10)
        self.sub_cmd = rospy.Subscriber('/velocity_command', velocity_data, self.cmd_callback)

        self.client.connect(BROKER, PORT, 60)

        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

    def on_connect(self, client, userdata, flags, rc):
        rospy.loginfo("Conectado ao broker MQTT")
        client.subscribe([(TOPIC_ENCODER_FRONT,0),(TOPIC_ENCODER_REAR,0)])

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            left = payload.get("left" , 0)
            right = payload.get("right" , 0)

            with lock:
                if msg.topic == TOPIC_ENCODER_FRONT:
                    encoder_msg.front_left_encoder_data = (left/1000.0)*0.118*0.10472
                    encoder_msg.front_right_encoder_data = (right/1000.0)*0.118*0.10472
                elif msg.topic == TOPIC_ENCODER_REAR:
                    encoder_msg.rear_left_encoder_data = (left / 1000.0)*0.118*0.10472
                    encoder_msg.rear_right_encoder_data = (right/1000.0)*0.118*0.10472

            self.pub_encoder.publish(encoder_msg)

        except Exception as e:
            rospy.logerr(f"Erro ao processar mensagem MQTT: {e}")

    def cmd_callback(self, msg):
        try:
            # Comando para FRONT
            data_front = {
                "left": int(msg.front_left_wheel * 1000),
                "right": int(msg.front_right_wheel * 1000)
            }
            self.client.publish(TOPIC_COMMAND_FRONT, json.dumps(data_front))

            # Comando para REAR
            data_rear = {
                "left": int(msg.rear_left_wheel * 1000),
                "right": int(msg.rear_right_wheel * 1000)
            }
            self.client.publish(TOPIC_COMMAND_REAR, json.dumps(data_rear))

        except Exception as e:
            rospy.logerr(f"Erro ao enviar comandos MQTT: {e}")

    def loop(self):
        self.client.loop_forever()

if __name__ == "__main__":
    rospy.init_node('mqtt_bridge')
    bridge = MQTTBridge()
    rospy.spin()
