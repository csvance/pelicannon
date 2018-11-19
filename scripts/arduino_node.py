#!/usr/bin/env python

import serial
import rospy
import struct
from threading import Thread, Event, RLock

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from std_msgs.msg import Bool, Float32


class ArduinoNode(object):

    SERIAL_HEADER = b"\xDE\xAD\xBE\xEF"

    def __init__(self):
        rospy.init_node('arduino')
        rospy.on_shutdown(self.shutdown)

        serial_device = rospy.get_param('serial_device', '/dev/ttyTHS2')
        serial_baud = rospy.get_param('serial_baud', 9600)
        self._serial = serial.Serial(serial_device, serial_baud, timeout=0)
        self._serial_lock = RLock()

        self._wakeup_event = Event()
        self._update_arduino_event = Event()
        self._new_position_event = Event()
        self._shutdown_event = Event()

        self._last_stepper = 0.
        self._last_fire = False
        self._last_spin = False
        self._last_position = 0.

        self._last_event_lock = RLock()

        self._tx_thread = Thread(target=self._tx_thread_proc)
        self._tx_thread.start()

        self._rx_thread = Thread(target=self._rx_thread_proc)
        self._rx_thread.start()

        rospy.Subscriber('/stepper', Float32,
                         self._stepper_callback, queue_size=100)
        rospy.Subscriber('/fire', Bool, self._fire_callback, queue_size=100)
        rospy.Subscriber('/spin', Bool, self._detect_callback, queue_size=100)

        self._position_publisher = rospy.Publisher(
            'position', Float32, queue_size=100)

    def _stepper_callback(self, r):
        if self._last_stepper != r.data
           with self._txrx_thread_lock:
                self._last_stepper = r.data
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _fire_callback(self, b):
        if self._last_fire != b.data:
            with self._txrx_thread_lock:
                self._last_fire = b.data
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _spin_callback(self, b):
        if self._last_spin != b.data:
            with self._txrx_thread_lock:
                self._last_spin = b.data
            self._update_arduino_event.set()
            self._wakeup_event.set()

    def _tx_latest():

        msg = None
        with self._last_event_lock:

            # Sync Header
            sync = ArduinoNode.SERIAL_HEADER

            # Stepper
            angular_z = struct.pack("f", self._last_stepper)

            # Spin
            spin = struct.pack("I", self._last_spin)

            # Fire
            fire = struct.pack("I", self._last_fire)

            msg = sync + angular_z + spin + fire

        with self._serial_lock:
            self._serial.write(msg)

    def _rx_handle_data(self, data):

        if len(data) == 0:
            return

        if data[0:4] != ArduinoNode.SERIAL_HEADER:
            self._rx_thread_sync()
            continue

        self._last_position = struct.unpack("f", data[4:8])
        self._new_position_event.set()
        self._wakeup_event.set()

    def _rx_thread_sync(self):
        rospy.loginfo(rospy.get_caller_id() + " Synchronizing stream...")

        while True:

            data = None

            with self._serial_lock:
                data = self._serial.read(1)

            if data[0] != "\xDE":
                continue

            with self._serial_lock:
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
            with self._serial_lock:
                self._rx_handle_data(self._serial.read(8))

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

            if self._new_position_event.is_set():
                self._position_publisher.publish(self._last_position)
                self._new_position_event.clear()

    def shutdown(self):
        self._shutdown_event.set()
        self._wakeup_event.set()
        self._serial.close()

if __name__ == "__main__":
    node = ArduinoNode()
    rospy.spin()
