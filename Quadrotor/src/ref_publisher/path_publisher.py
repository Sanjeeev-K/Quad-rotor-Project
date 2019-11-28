#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import sys
import select
import tf
import numpy as np

class Path_Publisher:

    def __init__(self):
        self.pub = rospy.Publisher('command/pose', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.publish_reference)
        self.path = [[0,0,0.2],[1.5,0,0.8],[2.5,3.0,0.8],[4.5,6.0,0.8]]
        self.index = 0

    def publish_reference(self, msg):
        pose = PoseStamped()

        pose.header.frame_id = "world"

        pose.pose.position.x = self.path[self.index][0]
        pose.pose.position.y = self.path[self.index][1]
        pose.pose.position.z = self.path[self.index][2]

        yaw = 0.0;
        pitch = 0.0;
        roll = 0.0;

        pose.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(yaw, pitch, roll, 'rzyx')
        pose.pose.orientation = Quaternion(*q)

        SAE = np.abs(msg.pose.position.x - self.path[self.index][0]) + \
                    np.abs(msg.pose.position.y - self.path[self.index][1]) + \
                    np.abs(msg.pose.position.z - self.path[self.index][2])

        if SAE < 0.1 and self.index < (len(self.path) - 1):
            self.index = self.index + 1

        self.pub.publish(pose)

if __name__ == '__main__':
    rospy.init_node('pose_publisher', anonymous=True)
    path_publisher = Path_Publisher()
    rospy.spin()
