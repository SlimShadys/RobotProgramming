#!/usr/bin/env python

import rospy
from std_msgs.msg import *

# This receives the data from 'talking_topic'.
# The msg variable is what 'talking_topic' sends us.
def listenCallback(msg):
    rospy.loginfo(F"Received data: {msg.data}")

def listener():
    # name = Name of the node
    # anonymous = If we create two of these nodes, one will get followed by a number
    #             (Example: subscriber_node_12849124)
    rospy.init_node("subscriber_node", anonymous=True)

    # name = Name of the topic
    # data_class = Which type we want to pass
    # callback = The function that the subscriber has call in order to gather
    #          = from 'talking_topic'
    rospy.Subscriber(name="talking_topic", data_class=String, callback=listenCallback)
    
    # Run the node continuously until we shut it down
    rospy.spin()

# Main function
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        print("Application stopped")
        pass

