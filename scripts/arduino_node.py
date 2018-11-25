#!/usr/bin/env python

import serial
import rospy
import struct
from threading import Thread, Event, RLock

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, Float32


class ArduinoNode(object):

    SERIAL_HEADER = b"\xDE\xAD\xBE\xEF"

    def __init__(self):
        rospy.init_node('arduino')
        rospy.on_shutdown(self.shutdown)

        serial_device = rospy.get_param('serial_device', '/dev/ttyTHS2')
        serial_baud = rospy.get_param('serial_baud', 9600)
        self._serial = serial.Serial(serial_device, serial_baud, timeout=0)

        self._wakeup_event = Event()
        self._update_arduino_event = Event()
        self._new_position_event = Event()
        self._shutdown_event = Event()

        self._last_stepper_speed = [0., 0., 0.]
        self._last_stepper_goal = [0., 0., 0.]

        self._last_fire = False
        self._last_spin = False

        self._last_event_lock = RLock()

        self._tx_thread = Thread(target=self._tx_thread_proc)
        self._tx_thread.start()

        self._rx_thread = Thread(target=self._rx_thread_proc)
        self._rx_thread.start()

        rospy.Subscriber('/stepper_speed', Vector3,
                         self._stepper_speed_callback, queue_size=100)
        rospy.Subscriber('/stepper_goal', Vector3,
                         self._stepper_goal_callback, queue_size=100)
        rospy.Subscriber('/fire', Bool, self._fire_callback, queue_size=100)
        rospy.Subscriber('/spin', Bool, self._detect_callback, queue_size=100)

        self._position_publisher = rospy.Publisher(
            'position', Vector3, queue_size=100)

    def _stepper_speed_callback(self, v):
        if self._last_stepper_speed != [v.x, v.y, v.z]
            with self._last_event_lock:
                self._last_stepper_speed = [v.x, v.y, v.z]
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _stepper_goal_callback(self, v):
        if self._last_stepper_goal != [v.x, v.y, v.z]
            with self._last_event_lock:
                self._last_stepper_goal = [v.x, v.y, v.z]
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _fire_callback(self, b):
        if self._last_fire != b.data:
            with self._last_event_lock:
                self._last_fire = b.data
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _spin_callback(self, b):
        if self._last_spin != b.data:
            with self._last_event_lock:
                self._last_spin = b.data
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _tx_latest():

        msg = None
        with self._last_event_lock:

            # Sync Header
            sync = ArduinoNode.SERIAL_HEADER

            # Speed for yaw and pitch movements
            speed_pitch = struct.pack("d", self._last_stepper_speed[0])
            speed_yaw = struct.pack("d", self._last_stepper_speed[2])

            # Spin
            spin = struct.pack("I", self._last_spin)

            # Fire
            fire = struct.pack("I", self._last_fire)

            # Enable goal mode
            goal = struct.pack("d", 1)

            # Goal pose
            goal_pitch = struct.pack("d", self._last_stepper_goal[0])
            goal_yaw = struct.pack("d", self._last_stepper_goal[2])

            msg = sync + speed_pitch + speed_yaw + spin + fire + goal + goal_pitch + goal_yaw

        self._serial.write(msg)

    def _rx_handle_data(self, data):

        if len(data) == 0:
            return

        if data[0:4] != ArduinoNode.SERIAL_HEADER:
            self._rx_thread_sync()
            continue

        euler = Vector3()
        euler.x = struct.unpack("d", data[4:8])
        euler.y = 0.
        euler.z = struct.unpack("d", data[8:12])
        self._position_publisher.publish(euler)

    def _rx_thread_sync(self):
        rospy.loginfo(rospy.get_caller_id() + " Synchronizing stream...")

        while True:

            data = self._serial.read(1)

            if data[0] != "\xDE":
                continue

            data += self._serial.read(7)

            if data[0:4] != ArduinoNode.SERIAL_HEADER[0:4]
               continue

            self._rx_handle_data(data)
            break

    def _rx_thread_proc(self):

        # Start receiving by synchronizing the serial stream
        self._rx_thread_sync()

        while True:
            # Read data from arduino
            buffer = ""
            self._rx_handle_data(self._serial.read(12))

            if self._shutdown_event.wait(0.001)
               return


    def _tx_thread_proc(self):
        while True:
            self._wakeup_event.wait()
            self._wakeup_event.clear()

            if self._shutdown_event.is_set():
                return

            if self._update_arduino_event.is_set():
                self._tx_latest()
                self._update_arduino_event.clear()

    def shutdown(self):
        self._shutdown_event.set()
        self._wakeup_event.set()
        self._serial.close()

if __name__ == "__main__":
    node = ArduinoNode()
    rospy.spin()
