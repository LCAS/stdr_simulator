#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from stdr_msgs.msg import RfidSensorMeasurementMsg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

import sys


rospy.init_node('molinillo')




# Node example class.
class formatParserRfid():

    def rfid_callback(self,msg):
        outMsg=''

        lalalal=RfidSensorMeasurementMsg()
        lalalal.

        separator=':'
             #   0:300833B2DDD9014000000010:-63:140:866900:342:3771850356
        fields=['0', msg.rfid_tags_ids, str(msg.rfid_tags_dbs),0,0]


        return outMsg



    def rfid_callback(self,msg):
        outMsg=0


        return outMsg

    def rfid_callback(self,msg):
        outMsg=0


        return outMsg

    # constructor.
    def __init__(self):

        # params
        self.rfid_dest_topic = '/rfid/rfid_detect'
        self.rfid_orig_topic = '/robot0/rfid_reader_0'
        self.odom_dest_topic = '/odom'
        self.odom_orig_topic = '/robot0/odom'
        self.laser_dest_topic = '/base_scan'
        self.laser_orig_topic = '/robot0/laser_0'

        # RFID!
        # publisher
        self.rfid_pub = rospy.Publisher(self.rfid_dest_topic, String,queue_size=10)
        # subscriber
        self.rfid_sub = rospy.Subscriber(self.rfid_orig_topic, RfidSensorMeasurementMsg,self.rfid_callback)

        # odom
        # publisher
        self.odom_pub = rospy.Publisher(self.odom_dest_topic, Odometry, queue_size=10)
        # subscriber
        self.odom_sub = rospy.Subscriber(self.odom_orig_topic, Odometry, self.odom_callback)

        # laser
        # publisher
        self.laser_pub = rospy.Publisher(self.laser_dest_topic, LaserScan, queue_size=10)
        # subscriber
        self.laser_sub = rospy.Subscriber(self.laser_orig_topic, LaserScan, self.laser_callback)



# Main function.
if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('format_parser_rfid')

    # Go to class functions that do all the heavy lifting. Do error checking.
    try:
        formatParserRfid()
    except rospy.ROSInterruptException: pass
