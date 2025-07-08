#!/usr/bin/env python3

"""Para roda:
rosrun seu_pacote imu_recorder.py \
  _raw_topic:=/imu/data_raw \
  _filt_topic:=/imu/data_filtered \
  _output_file:=comparacao.csv
"""
import rospy
from sensor_msgs.msg import Imu
import csv

# Listas para armazenar as leituras
raw_data = []
filtered_data = []

def raw_cb(msg):
    if len(raw_data) < 15:
        raw_data.append({
            'stamp': msg.header.stamp.to_sec(),
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
            # não há orientação no raw, mas mantenho campos vazios para encaixe
            'qx': None, 'qy': None, 'qz': None, 'qw': None,
        })
    check_done()

def filtered_cb(msg):
    if len(filtered_data) < 15:
        filtered_data.append({
            'stamp': msg.header.stamp.to_sec(),
            'ax': msg.linear_acceleration.x,
            'ay': msg.linear_acceleration.y,
            'az': msg.linear_acceleration.z,
            'gx': msg.angular_velocity.x,
            'gy': msg.angular_velocity.y,
            'gz': msg.angular_velocity.z,
            'qx': msg.orientation.x,
            'qy': msg.orientation.y,
            'qz': msg.orientation.z,
            'qw': msg.orientation.w,
        })
    check_done()

def check_done():
    # assim que tivermos 15 de cada, salva e encerra
    if len(raw_data) >= 15 and len(filtered_data) >= 15:
        save_and_exit()

def save_and_exit():
    filename = rospy.get_param('~output_file', 'imu_comparison.csv')
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        # cabeçalho
        writer.writerow([
            'source','stamp',
            'ax','ay','az',
            'gx','gy','gz',
            'qx','qy','qz','qw'
        ])
        # escreve raws
        for r in raw_data:
            writer.writerow([
                'raw', r['stamp'],
                r['ax'], r['ay'], r['az'],
                r['gx'], r['gy'], r['gz'],
                r['qx'] or '', r['qy'] or '', r['qz'] or '', r['qw'] or ''
            ])
        # escreve filtered
        for r in filtered_data:
            writer.writerow([
                'filtered', r['stamp'],
                r['ax'], r['ay'], r['az'],
                r['gx'], r['gy'], r['gz'],
                r['qx'], r['qy'], r['qz'], r['qw']
            ])
    rospy.loginfo("Dados salvos em %s", filename)
    rospy.signal_shutdown("Coleta concluída")

def main():
    rospy.init_node('imu_data_recorder')
    # tópicos — ajuste se os seus forem diferentes
    raw_topic      = rospy.get_param('~raw_topic', '/imu/data_raw')
    filt_topic     = rospy.get_param('~filt_topic', '/imu/data_filtered')

    rospy.Subscriber(raw_topic,  Imu, raw_cb)
    rospy.Subscriber(filt_topic, Imu, filtered_cb)

    rospy.loginfo("Aguardando 15 amostras em %s e %s ...", raw_topic, filt_topic)
    rospy.spin()

if __name__ == '__main__':
    main()
