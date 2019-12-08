# -*- coding: utf-8 -*-
import datetime
import json
import math
import os
import ssl
from threading import Lock

import pytz

import paho.mqtt.client as mqtt

import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

from roomba_ros_bridge.logging import getLogger
logger = getLogger(__name__)


class AttrsBridge(object):
    def __init__(self):
        super(AttrsBridge, self).__init__()
        self.__params = rospy.get_param('~')
        self.__client = None

        entity_type = self.__params['roboticbase']['entity_type']
        entity_id = self.__params['roboticbase']['entity_id']

        self.__mqtt_attrs_topic = '/{}/{}/attrs'.format(entity_type, entity_id)
        rospy.Subscriber('/odom', Odometry, self._on_odom_receive, queue_size=1)
        self.__tz = pytz.timezone(self.__params['timezone'])
        self.__send_delta_ms = self.__params['thresholds']['send_delta_millisec']
        self.__prev_ms = datetime.datetime.now(self.__tz)
        self.__lock = Lock()

    def connect(self):
        logger.infof('try to Connect mqtt broker, host={}', self.__params['mqtt']['host'])
        self.__client = mqtt.Client(protocol=mqtt.MQTTv311)
        self.__client.on_connect = self._on_connect

        if 'use_ca' in self.__params['mqtt'] and self.__params['mqtt']['use_ca'] and 'cafile' in self.__params['mqtt']:
            cafile = self.__params['mqtt']['cafile'].strip()
            if len(cafile) > 0 and os.path.isfile(cafile):
                self.__client.tls_set(cafile, tls_version=ssl.PROTOCOL_TLSv1_2)

        if 'username' in self.__params['mqtt'] and 'password' in self.__params['mqtt']:
            username = self.__params['mqtt']['username'].strip()
            password = self.__params['mqtt']['password'].strip()
            if len(username) > 0 and len(password) > 0:
                self.__client.username_pw_set(username, password)

        self.__client.connect(self.__params['mqtt']['host'], port=self.__params['mqtt']['port'], keepalive=60)
        self.__client.loop_start()

        rospy.on_shutdown(self._on_shutdown)
        return self

    def start(self):
        logger.infof('AttrsBridge start')
        rospy.spin()
        logger.infof('AttrsBridge finish')

    def _on_connect(self, client, userdata, flags, response_code):
        logger.infof('connected to mqtt broker, status={}', response_code)

    def _on_shutdown(self):
        if self.__client:
            self.__client.loop_stop()
            self.__client.disconnect()

    def _on_odom_receive(self, odom):
        now = datetime.datetime.now(self.__tz)
        if now >= self.__prev_ms + datetime.timedelta(milliseconds=self.__send_delta_ms) and self.__lock.acquire(False):
            self.__prev_ms = now

            pos = odom.pose.pose.position
            qt = odom.pose.pose.orientation
            theta = math.degrees(euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])[2])

            message = {
                'time': now.isoformat(),
                'x': pos.x,
                'y': pos.y,
                'theta': theta,
            }

            if self.__client:
                self.__client.publish(self.__mqtt_attrs_topic, json.dumps(message))
            self.__lock.release()
