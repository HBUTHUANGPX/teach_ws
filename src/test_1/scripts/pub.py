#! /usr/bin/env python

import rospy
from test_1.msg import self_msg_1
from test_1.msg import self_msg_2
from test_1.msg import self_msg_3

if __name__ == "__main__":
    rospy.init_node("haha")
    pub = rospy.Publisher("haha_chat",self_msg_1,queue_size=1)
    pub2 = rospy.Publisher("haha_chat2",self_msg_2,queue_size=1)
    msg = self_msg_1()
    msg.name = "nh"
    msg.age = 1
    msg.height = 0.0
    
    msg2 = self_msg_2()
    msg2.wsnd.name = "wsnd"
    msg2.wsnd.age = 1
    msg2.wsnd.height = 0.0
    msg2.wsned.pose.position.x = 1.
    msg2.wsned.pose.position.y = 2.
    msg2.wsned.pose.position.z = 3.
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        pub.publish(msg)
        pub2.publish(msg2)
        msg2.wsnd.height +=0.001
        msg.height += 0.001
        rate.sleep()
        # FIFO first in fisrt out DMA CAN