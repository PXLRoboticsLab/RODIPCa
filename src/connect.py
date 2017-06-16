#!/usr/bin/env python
import cv2
import urllib
import numpy as np
from sensor_msgs.msg import Image
import sys
import rospy
from cv_bridge import CvBridge, CvBridgeError
import argparse


class Connect():
    _url = None
    _name = None
    _password = None
    _topic = None
    _fps = 30
    _screen = None

    def __init__(self, url, name, password, topic, fps, screen):
        self._url = url
        self._name = name
        self._password = password
        self._topic = topic
        self._fps = fps
        self._screen = screen

        if "https://" in self._url:
            self._url = self._url[8:]
        elif "http://" in self._url:
            self._url = self._url[7:]

        try:
            stream = urllib.urlopen('http://' + self._name + ':' + self._password + '@' + self._url)
        except:
            rospy.logerr('Unable to open camera stream: ' + str(self._url))
            sys.exit()

        image_pub = rospy.Publisher(self._topic, Image, queue_size=self._fps)
        rate = rospy.Rate(self._fps)
        bridge = CvBridge()
        bytes = ''

        print('Started connection on: ' + self._url)

        while not rospy.is_shutdown():
            bytes += stream.read(1024)
            a = bytes.find('\xff\xd8')
            b = bytes.find('\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b + 2]
                bytes = bytes[b + 2:]
                i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                image_pub.publish(bridge.cv2_to_imgmsg(i, 'bgr8'))

                if screen:
                    cv2.imshow('Camera stream', i)
                if cv2.waitKey(1) == 27:
                    exit(0)

                rate.sleep()


if __name__ == '__main__':
    rospy.init_node('dlink_connector')
    parser = argparse.ArgumentParser()

    parser.add_argument('url', type=str,
                        help='The URL of the D\'Link IP camera stream.')
    parser.add_argument('--name', type=str,
                        help='The login name of the camera.', default="admin")
    parser.add_argument('--password', type=str,
                        help='The login password of the camera.', default="")
    parser.add_argument('--topic', type=str,
                        help='The name of the rostopic', default="focus_vision/image/compressed")
    parser.add_argument('--fps', type=int,
                        help='The max frame rate of the camera.', default=30)
    parser.add_argument('--screen', action='store_true',
                        help='Show a GUI of the camera stream.')
    args = parser.parse_args()
    Connect(args.url, args.name, args.password, args.topic, args.fps, args.screen)

