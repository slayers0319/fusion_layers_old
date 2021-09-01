#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
import random

data_x = 0.0
data_y = 0.0
data_yaw = 0.0

def talker():
    
    global name, data_x, data_y, data_yaw, data
    pub = rospy.Publisher('fusion_data', String, queue_size=10)
    rospy.init_node('markertest_pub', anonymous=True)
    rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        data = String()
        n=input()
        if n==-1:
            data.data = "person,1.2,0.87,R"
            rospy.loginfo(data)
            pub.publish(data)
            break
        elif n==1:
            data.data = "person,1.2,0.87,R,obstacle,0.5,0.66,G"
            rospy.loginfo(data)
            pub.publish(data)
        elif n==2:
            data.data = "person,1.2,0.87,R,obstacle,0.5,0.66,G,person,1.0,1.3,B"
            rospy.loginfo(data)
            pub.publish(data)
        elif n==3:
            data.data = "person,1.2,0.87,R,obstacle,0.5,0.66,G,person,1.0,1.3,B,person,-1.2,0.87,R,obstacle,-0.5,0.66,G,person,-1.0,1.3,B"
            rospy.loginfo(data)
            pub.publish(data)
        else:
            for i in range(n):
                rate.sleep()
                name = "obstacle"
                data_x = random.uniform(0,0.5)
                data_y = random.uniform(0.5,1.0)
                data.data = name+","+str(data_x)+","+str(data_y)+","+"obstacle"+","+str(random.uniform(-0.5,0.0))+","+str(random.uniform(0.5,1.0))
                rospy.loginfo(data)
                pub.publish(data)
            

        rospy.loginfo("-------------")
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
