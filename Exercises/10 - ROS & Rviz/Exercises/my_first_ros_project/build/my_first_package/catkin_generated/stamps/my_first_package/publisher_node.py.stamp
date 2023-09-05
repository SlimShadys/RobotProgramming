#!/usr/bin/env python

import rospy
from std_msgs.msg import *

def talk_to_me():
    # name = Name of the node
    # anonymous = If we create two of these nodes, one will get followed by a number
    #             (Example: publisher_node_12849124)
    rospy.init_node('publisher_node', anonymous=True)

    # name = Name of the topic
    # data_class = Which type we want to pass
    # queue_size = How many messages we have in queue before starting deleting them
    pub = rospy.Publisher(name="talking_topic", data_class=String, queue_size=10)

    # This determines how long ROS sleeps before waking up again
    # If set to 0, it means it's always active
    # If set to 1, it means it sleeps for 1 second
    rate = rospy.Rate(hz=1)

    # Log a message into ROS
    rospy.loginfo("'publisher_node' started successfully.")

    while not rospy.is_shutdown():
        message = F"Hi! Here's the time: {rospy.get_time()}"
        pub.publish(message)
        rate.sleep()

# Main function
if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSException:
        print("'publisher_node' stopped successfully.")
        pass

