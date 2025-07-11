#!/usr/bin/env python3
import rospy
import time
from robot_base_controller.msg import velocity_data, encoder_data
import matplotlib.pyplot as plt

class PIDTester:
    def __init__(self):
        rospy.init_node('pid_tester')

        self.pub_cmd = rospy.Publisher('/velocity_command', velocity_data, queue_size=1)
        rospy.Subscriber('/encoder_data', encoder_data, self.encoder_callback)

        self.data_time = []
        self.data_rear_left = []
        self.data_rear_right = []

        self.recording = False

    def encoder_callback(self, msg):
        if self.recording:
            t = time.time()
            self.data_time.append(t)
            self.data_rear_left.append(msg.rear_left_encoder_data)
            self.data_rear_right.append(msg.rear_right_encoder_data)

    def send_command(self, speed):
        cmd = velocity_data()
        cmd.rear_left_wheel = speed
        cmd.rear_right_wheel = speed
        cmd.front_left_wheel = 0.0
        cmd.front_right_wheel = 0.0
        self.pub_cmd.publish(cmd)

    def run_phase(self, pre_speed, post_speed, pre_duration, post_duration, label):
        total_duration = pre_duration + post_duration
        rospy.loginfo(
            f"Iniciando fase '{label}': "
            f"{pre_duration}s a {pre_speed} m/s -> {post_duration}s a {post_speed} m/s"
        )
        self.data_time = []
        self.data_rear_left = []
        self.data_rear_right = []

        self.recording = True
        t_start = time.time()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            elapsed = time.time() - t_start
            if elapsed < pre_duration:
                self.send_command(pre_speed)
            elif elapsed < total_duration:
                self.send_command(post_speed)
            else:
                break
            rate.sleep()
        self.recording = False

        if not self.data_time:
            rospy.logwarn("Nenhum dado coletado nesta fase!")
            return

        t0 = self.data_time[0]
        times = [x - t0 for x in self.data_time]

        plt.figure()
        plt.plot(times, self.data_rear_left, label='Rear Left Encoder')
        plt.plot(times, self.data_rear_right, label='Rear Right Encoder')
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.title(f'Response - {label}')
        plt.legend()
        plt.grid(True)

        filename = f"./Work_2025/jetson/robot_ws/response_{label.replace(' ', '_')}.png"
        plt.savefig(filename)
        plt.close()
        rospy.loginfo(f"Gráfico salvo como {filename}")

    def run(self):
        # Fase subida: 2s parado -> 5s em 6.5
        # self.run_phase(pre_speed=0.0, post_speed=6.5, pre_duration=2, post_duration=10, label='subida')
        # rospy.sleep(2)
        # Fase descida: 2s em 6.5 -> 5s em 0
        self.run_phase(pre_speed=6.5, post_speed=0.0, pre_duration=5, post_duration=10, label='descida')
        rospy.loginfo("Teste concluído.")

if __name__ == "__main__":
    tester = PIDTester()
    tester.run()
