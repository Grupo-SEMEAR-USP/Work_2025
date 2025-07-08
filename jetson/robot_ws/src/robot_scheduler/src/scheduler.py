#!/usr/bin/env python3

import rospy
from robot_scheduler.scheduler_communication import SchedulerCommunication

def main():
    rospy.init_node('scheduler_test_node')
    comm = SchedulerCommunication()

    rospy.loginfo("Sending command to navigation...")
    response = comm.call_function("navigation", "go_to:point_A", timeout=10)

    if response:
        rospy.loginfo(f"Response from {response.key}: {response.status} - {response.payload}")
    else:
        rospy.loginfo("No response received.")

if __name__ == "__main__":
    main()
