#!/usr/bin/env python

import rospy
from my_first_package.srv import multiplier, multiplierResponse

def callback(request):
    return multiplierResponse(request.a * request.b)

def multiply():
    rospy.init_node('multiplier_service')
    service = rospy.Service("multiplier", service_class=multiplier, handler=callback)
    rospy.spin()
    rospy.loginfo("Multiplier service started successfully.")

# Main function
if __name__ == '__main__':
    multiply()