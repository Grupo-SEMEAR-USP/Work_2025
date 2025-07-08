#!/usr/bin/env python3

import rospy
from robot_scheduler.msg import SchedulerCommand, SchedulerResponse
from threading import Event

class SchedulerCommunication:
    def __init__(self):
        self.publisher = rospy.Publisher('/robot_scheduler/commands', SchedulerCommand, queue_size=10)
        self.subscriber = rospy.Subscriber('/robot_scheduler/responses', SchedulerResponse, self._response_callback)
        self._response_event = Event()
        self._last_response = None

    def send_command(self, key, payload, continue_scheduler=False):
        msg = SchedulerCommand()
        msg.key = key
        msg.payload = payload
        msg.continue_scheduler = continue_scheduler
        self.publisher.publish(msg)

    def _response_callback(self, msg):
        self._last_response = msg
        self._response_event.set()

    def wait_for_response(self, timeout=None):
        self._response_event.clear()
        completed = self._response_event.wait(timeout)
        if completed:
            return self._last_response
        return None

    def call_function(self, key, payload, timeout=None):
        self.send_command(key, payload, continue_scheduler=False)
        return self.wait_for_response(timeout)
