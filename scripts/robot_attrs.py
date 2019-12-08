#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from roomba_ros_bridge.attrs_bridge import AttrsBridge

NODE_NAME = 'robot_attrs'


def main():
    try:
        rospy.init_node(NODE_NAME)
        AttrsBridge().connect().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
