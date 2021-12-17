#!/usr/bin/env python

import rospy
import dynamic_reconfigure.server
from pva_tracker.cfg import PVA_Ground_TrackerConfig


def severCallback(config, level):
    return config


if __name__ == '__main__':
    rospy.init_node('ground_reconfigure_server', anonymous=False)
    server = dynamic_reconfigure.server.Server(PVA_Ground_TrackerConfig, severCallback)
    rospy.spin()