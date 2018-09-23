#!/usr/bin/env python
import rospy
from smart_eef_control.control_node import ControlNode


if __name__ == '__main__':
    rospy.init_node('smart_eef_control')

    node = ControlNode.from_dict(rospy.get_param('eef_control'))

    while not rospy.is_shutdown():
        pass

    node.stop()