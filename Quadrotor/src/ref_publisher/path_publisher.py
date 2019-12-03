#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
import sys
import select
import tf
import numpy as np

class Path_Publisher:

    def __init__(self, world_file_name):
        self.pub = rospy.Publisher('command/pose', PoseStamped, queue_size=10)
        self.sub = rospy.Subscriber('/ground_truth_to_tf/pose', PoseStamped, self.publish_reference)
        self.world_file_name = world_file_name
        if self.world_file_name == 'world_test.world':
            self.path = [[0,0,0.2],[1.5,0,0.8],[2.5,3.0,0.8],[4.5,6.0,0.8]]
        elif self.world_file_name == 'world_demo_corridor.world':
            self.path = [[0,0,0.2],[4.0,-0.3,1.1],[6.0,0.2,1],[8.0,1.8,0.9],[5.2,4.5,1.3],[1.3,4.2,1.0],[0.2,5.8,1.2],[1.2,8.8,1.1],[4.0,9.1,0.7],[6.0,8.6,0.5],[8.0,8.5,0.2]]
        elif self.world_file_name == 'world_demo_xy.world':
            self.path = [[0,0,0.2],[2.0,1.5,0.5],[2.5,3.0,0.8],[4.0,4.5,0.7],[4.5,6.0,0.8],[3.0,7.5,0.9],[2.5,9.0,0.8],[4.0,10.5,0.9],[4.5,12.0,0.8],[4.5,13.0,0.5],[4.5,14.0,0.2]]
        elif self.world_file_name == 'world_demo_xyz.world':
            self.path = [[0,0,0.2],[2.0,1.5,0.8],[2.5,3.0,1.3],[4.0,4.5,0.7], [4.5,6.0,1.3], [3.0,7.5,0.9],[2.5,9.0,1.3],[4.0,10.5,0.9],[4.5,12.0,1.3],[4.5,13.0,0.5],[4.5,14.0,0.2]]
        elif self.world_file_name == 'world_demo_circle.world':
            self.path = [[0,0,0.2],[2.0,0.7,0.6],[2.5,2.0,0.8],[4.0,3.5,0.7],[4.5,5.0,0.8],[5.0,8.0,1.0],[1.0,10.5,0.8],[-3.0,9.0,0.9],[-4.0,7.0,0.8],[-5.0,5.0,0.7],[-5.5,3.0,0.8],[-5.0,1.5,0.6],[0,0,0.2]]
        else:
            rospy.loginfo('The world file name doe not match')
            rospy.signal_shutdown('Quit')

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
    world_file_name = rospy.get_param('world_file_name')
    path_publisher = Path_Publisher(world_file_name)
    rospy.spin()
