#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import struct
import rospy
from smbus2 import SMBus, i2c_msg
from robot_perception.msg import UltrasonicDistances

# Configurações
I2C_BUS = 1
ARDUINO_ADDR = 0x0a
NUM_SENSORS = 5
PKT_LEN = NUM_SENSORS * 4
PUB_TOPIC = '/ultrasonic_distances'
PUB_RATE_HZ = 50

def read_ultrasonic(bus):
    read = i2c_msg.read(ARDUINO_ADDR, PKT_LEN)
    bus.i2c_rdwr(read)
    raw = bytes(read)
    return list(struct.unpack('<' + 'f'*NUM_SENSORS, raw))

def main():
    rospy.init_node('i2c_ultrasonic')
    pub = rospy.Publisher(PUB_TOPIC, UltrasonicDistances, queue_size=10)
    rate = rospy.Rate(PUB_RATE_HZ)

    with SMBus(I2C_BUS) as bus:
        while not rospy.is_shutdown():
            try:
                readings = read_ultrasonic(bus)
                msg = UltrasonicDistances()
                msg.front_left  = readings[3]
                msg.front_right = readings[2]
                msg.left        = readings[1]
                msg.right       = readings[0]
                msg.rear        = readings[4]
                pub.publish(msg)
            except Exception as e:
                rospy.logwarn_throttle(5.0, f'I2C read failed: {e}')
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
