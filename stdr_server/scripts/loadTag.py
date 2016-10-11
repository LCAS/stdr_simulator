#!/usr/bin/env python


import rospy
from stdr_msgs.msg import RfidTag
from stdr_msgs.srv import AddRfidTag
import sys

if len(sys.argv)!=5:
    print "missing arguments"
    exit

rospy.init_node('lalala')
rospy.wait_for_service('/stdr_server/add_rfid_tag')
add_rfid_tag=  rospy.ServiceProxy('/stdr_server/add_rfid_tag', AddRfidTag)

tag=RfidTag()
tag.tag_id=sys.argv[1]
tag.pose.x=float(sys.argv[2])
tag.pose.y=float(sys.argv[3])
tag.pose.theta=float(sys.argv[4])

add_rfid_tag(tag)
