#!/usr/bin/python
import rospy
import tf
from geometry_msgs.msg import *
from nav_msgs.msg import *

class VoRelay:
    def __init__(self):
        self.baseId = rospy.get_param('~base_id', 'base_link') # base link
        self.odomId = rospy.get_param('~odom_id', 'odom') # odom link
        self.pub_tf = rospy.get_param('~pub_tf',True)
        self.odom_topic = rospy.get_param('~odom_topic','odom')
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)

        if self.pub_tf:
            self.tf_broadcaster = tf.TransformBroadcaster()


    def odom_relay(self,vo_msg):
        new_odom = vo_msg
        new_odom.header.frame_id = self.odomId
        new_odom.child_frame_id = self.baseId

        # fix z axies
        new_odom.pose.pose.position.z = 0.0

        # lock raw and pitch
        old_quat = (vo_msg.pose.pose.orientation.x,vo_msg.pose.pose.orientation.y,
                    vo_msg.pose.pose.orientation.z,vo_msg.pose.pose.orientation.w)
        (raw,pitch,yaw) = tf.transformations.euler_from_quaternion(old_quat)
        new_quat = tf.transformations.quaternion_from_euler(0,0,yaw)
        new_odom.pose.pose.orientation.x =  new_quat[0]
        new_odom.pose.pose.orientation.y =  new_quat[1]
        new_odom.pose.pose.orientation.z =  new_quat[2]
        new_odom.pose.pose.orientation.w =  new_quat[3]
        self.odom_pub.publish(new_odom)

        if self.pub_tf:
            new_pose = (new_odom.pose.pose.position.x,new_odom.pose.pose.position.y,0)
            self.tf_broadcaster.sendTransform( new_pose, new_quat, 
            vo_msg.header.stamp, self.baseId, self.odomId)


if __name__=="__main__":
    rospy.init_node('vodom_relay')
    vo_relay = VoRelay()
    rospy.Subscriber('/camera/odom/sample',Odometry,vo_relay.odom_relay)
    rospy.spin()
    