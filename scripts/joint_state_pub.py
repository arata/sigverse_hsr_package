#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import message_filters

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist


class JointStatePublisher:
    def __init__(self):

        self.tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(self.tf_buffer)

        joint_sub = rospy.Subscriber("/hsrb/joint_states", JointState, self.joint_callback)
        self.joint_pub = rospy.Publisher("/hsrb/joint_states_moveit", JointState, queue_size=10)

    def joint_callback(self, msg):
        try:
            trans = self.tf_buffer.lookup_transform("odom", 'base_footprint', rospy.Time())
            odom_x_joint = trans.transform.translation.x
            odom_y_joint = trans.transform.translation.y

            quaternion = trans.transform.rotation
            now_odom_t_joint = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))[-1]
            # if now_odom_t_joint < 0:
            #     now_odom_t_joint += 3.14 * 2

            print(odom_x_joint, odom_y_joint, now_odom_t_joint)

            msg.name += ["odom_x", "odom_y", "odom_t"]
            msg.position += (odom_x_joint, odom_y_joint, now_odom_t_joint)
            msg.velocity += (0.0, 0.0, 0.0)
            msg.effort += (0.0, 0.0, 0.0)

            self.joint_pub.publish(msg)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            import traceback
            traceback.print_exc()

        
if __name__ == "__main__":
    rospy.init_node("add_odom_joint")
    JointStatePublisher()
    rospy.spin()
