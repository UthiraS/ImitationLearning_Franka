#!/usr/bin/env python3
# license removed for brevity
import math
import random
import rospy
from std_msgs.msg import Float64

def talker():
    
    pub1 = rospy.Publisher('/vsbot/joint1_velocity_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/vsbot/joint2_velocity_controller/command', Float64, queue_size=10)
    
    rospy.init_node('joint_vel_talker', anonymous=True)
    rate = rospy.Rate(1) # meaning 1 message published in 1 sec
    random.seed()     
    
    #i = 0
    while not rospy.is_shutdown():
        
        q1_vel = random.uniform(-0.5, 0.5) #math.sin(i)#
        q2_vel = random.uniform(-0.5, 0.5)
        #rospy.loginfo(q1_vel)
        #rospy.loginfo(q2_vel)
        pub1.publish(q1_vel)
        pub2.publish(q2_vel)
        rate.sleep()
        #i += 0.01

if __name__ == '__main__':
    try:
        talker()
        rospy.sleep(30)
    except rospy.ROSInterruptException:
        pass
