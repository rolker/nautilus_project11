#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped

import socket
import json
import tf.transformations
import math

rospy.init_node('nautilus_nav')

port = rospy.get_param('~port', 14202)
frame_id = rospy.get_param('~frame_id', 'project11/nautilus')

position_pub = rospy.Publisher('project11/nautilus/position',NavSatFix,queue_size=10)

orientation_pub = rospy.Publisher('project11/nautilus/orientation',Imu,queue_size=10)
velocity_pub = rospy.Publisher('project11/nautilus/velocity',TwistWithCovarianceStamped,queue_size=10)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', port))

while not rospy.is_shutdown():
  data = sock.recv(2048)
  d = json.loads(data)

  ts = rospy.Time.from_sec(d['time'])
  
  nsf = NavSatFix()
  nsf.header.stamp = ts
  nsf.header.frame_id = frame_id
  nsf.latitude = d['latitude']
  nsf.longitude = d['longitude']
  nsf.altitude = d['height']+d['heave']
  position_pub.publish(nsf)

  imu = Imu()                  
  imu.header.stamp = ts
  imu.header.frame_id = frame_id
  q = tf.transformations.quaternion_from_euler(math.radians(90.0-d['heading']), math.radians(-d['pitch']), math.radians(d['roll']), 'rzyx')
  
  imu.orientation.x = q[0]
  imu.orientation.y = q[1]
  imu.orientation.z = q[2]
  imu.orientation.w = q[3]
  imu.angular_velocity.x = math.radians(d['rate_roll'])
  imu.angular_velocity.y = math.radians(-d['rate_pitch'])
  imu.angular_velocity.z = math.radians(-d['rate_heading'])

  orientation_pub.publish(imu)

  twcs = TwistWithCovarianceStamped()
  twcs.header.stamp = ts
  twcs.header.frame_id = frame_id
  twcs.twist.twist.linear.x = d['vel_east']
  twcs.twist.twist.linear.y = d['vel_north']
  twcs.twist.twist.linear.z = -d['vel_down']
  velocity_pub.publish(twcs)