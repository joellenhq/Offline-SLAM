#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import PointCloud2
import time

time1 = time.time()
status = False

 
def callback(data):
    global time1, status
    # reset timer
    time1 = time.time()
    # set lidar_status message to True
    status = True


def lidar_status_checker():
    # imu status
    global status, time1

    pub = rospy.Publisher('lidar_status', Bool, queue_size=3)
    rospy.init_node('lidar_status_checker', anonymous=True)
    # subscribe imu message
    rospy.Subscriber("velodyne_points", PointCloud2, callback)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        if (time.time() - time1) > 1:
            # if there wasn't any message for two seconds set imu state to False
            status = False

        # rospy.loginfo(status)
        pub.publish(status)
        rate.sleep()


if __name__ == '__main__':
    try:
        lidar_status_checker()
    except rospy.ROSInterruptException:
        pass
