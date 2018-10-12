#!/usr/bin/env python

import signal
import rospy
import time
import math

from jetson_tensorrt.msg import ClassifiedRegionsOfInterest
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

class PelicannonNode(object):

    def __init__(self):
        signal.signal(signal.SIGINT, self.shutdown)
        signal.signal(signal.SIGTERM, self.shutdown)

        rospy.init_node('pelicannon')

        self._detect_window_track = rospy.get_param('detect_window_search', 10)
        self._detect_window_spin = rospy.get_param('detect_window_spin', 3)
        self._detect_window_fire = rospy.get_param('detect_window_fire', 0.5)

        self._delta_angle_fire = rospy.get_param('delta_angle_fire', math.pi/16.)

        self._last_detection_time = 0.0

        rospy.Subscriber('/detector/detections', ClassifiedRegionsOfInterest, self._detect_callback, queue_size=2)

        self._publisher = rospy.Publisher('stepper', Vector3, queue_size=100)
        self._publisher = rospy.Publisher('fire', Bool, queue_size=100)
        self._publisher = rospy.Publisher('spin', Bool, queue_size=100)

    def _detect_callback(regions):
        pass

    def shutdown(self):
        self._shutdown_event.set()
        self._wakeup_event.set()
