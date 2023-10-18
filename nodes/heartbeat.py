#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from project11_msgs.msg import Heartbeat, KeyValue


def navCallback(msg: NavSatFix):
  hb = Heartbeat()
  hb.header.stamp = msg.header.stamp
  kv = KeyValue()
  kv.key = 'latitude'
  kv.value = str(msg.latitude)
  hb.values.append(kv)
  kv = KeyValue()
  kv.key = 'longitude'
  kv.value = str(msg.longitude)
  hb.values.append(kv)

  heartbeat_pub.publish(hb)

rospy.init_node('nautilus_heartbeat')


heartbeat_pub = rospy.Publisher('project11/heartbeat', Heartbeat, queue_size=1)

nav_sub = rospy.Subscriber('sensors/seapath/fix', NavSatFix, navCallback)

rospy.spin()
