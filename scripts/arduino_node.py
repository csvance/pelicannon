#!/usr/bin/env python

import serial
import signal
import rospy
import struct
from threading import Thread, Event, Lock

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from std_msgs.msg import Bool, Float32

class ArduinoNode(object):
    def __init__(self):
        rospy.init_node('arduino')

        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        serial_device = rospy.get_param('serial_device', '/dev/ttyTHS2')
        serial_baud = rospy.get_param('serial_baud', 9600)
        self._serial = serial.Serial(serial_device, serial_baud)

        self._txrx_thread_lock = Lock()
        self._wakeup_event = Event()
        self._shutdown_event = Event()

        self._last_stepper = Float32()
        self._last_stepper.data = 0.

        self._last_fire = False
        self._last_spin = False

        self._txrx_thread = Thread(target=self._txrx_thread)
        self._txrx_thread.start()

        rospy.Subscriber('/stepper', Float32, self._stepper_callback, queue_size=100)
        rospy.Subscriber('/fire', Bool, self._fire_callback, queue_size=100)
        rospy.Subscriber('/spin', Bool, self._detect_callback, queue_size=100)

    def _stepper_callback(self, r):
        if self._last_stepper.data != r.data
            with self._txrx_thread_lock:
                self._last_stepper = r
            self._wakeup_event.set()

    def _fire_callback(self, b):
        if self._last_fire.data != b.data
            with self._txrx_thread_lock:
                self._last_fire = b
            self._wakeup_event.set()

    def _spin_callback(self, b):
        if self._last_spin.data != b.data:
            with self._txrx_thread_lock:
                self._last_spin = b
            self._wakeup_event.set()

    def _tx_latest():
        # Stepper
        angular_z = struct.pack("f", self._last_stepper.data)

        # Spin
        spin = struct.pack("I", self._last_spin.data)

        # Fire
        fire = struct.pack("I", self._last_fire.data)

        msg = angular_z + spin + fire
        self._serial.write(msg)

    def _txrx_thread_proc(self):
        while True:
            self._wakeup_event.wait()
            self._wakeup_event.clear()

            if self._shutdown_event.is_set():
                return

            with self._txrx_thread_lock:
                self._tx_latest()

    def shutdown(self):
        self._shutdown_event.set()
        self._wakeup_event.set()
