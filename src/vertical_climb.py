#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import math
import geometry_msgs.msg


def takeoff_command():
    
    cmd = geometry_msgs.msg.Twist()
 
    
    #linear:{x: 0, y: 0, z: 5.8}, ?angular: {x: 0, y: 0, z: 0}

    cmd.linear.x =0
    cmd.linear.y =0
    cmd.linear.z =4
    

    cmd.angular.x =0
    cmd.angular.y =0
    cmd.angular.z =0
   
    return cmd

def hover_command():
    
    cmd = geometry_msgs.msg.Twist()
 

    cmd.linear.x =0
    cmd.linear.y =0
    cmd.linear.z =0
    

    cmd.angular.x =0
    cmd.angular.y =0
    cmd.angular.z =0
   
    return cmd



    
def open_loop():
    rospy.init_node('open_loop', anonymous=True)
    quad_vel = rospy.Publisher('cmd_vel',geometry_msgs.msg.Twist ,queue_size=1)
    rate = rospy.Rate(10) # 10hz
    
    n=0
    print("climbing")
    while not rospy.is_shutdown():
        quad_vel.publish(takeoff_command())
        if n>=30:
           quad_vel.publish(hover_command())
           print("hovering")
        n=n+1
        rate.sleep()
     
    

 

if __name__ == '__main__':
	try:
		open_loop()
	except rospy.ROSInterruptException:
		pass
