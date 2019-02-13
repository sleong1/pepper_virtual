#!/usr/bin/env python
import rospy
from tf import TransformListener
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3, Quaternion

from sensor_msgs.msg import PointCloud2
rospy.init_node('asdfasdf')
tl = TransformListener()

pc = rospy.wait_for_message('/cloudr', PointCloud2)

transform_right_to_front = tl.lookupTransform('SurroundingRightLaser_frame', 'SurroundingFrontLaser_frame', rospy.Time.now())
ts = TransformStamped()
ts.transform.translation = Vector3(*transform_right_to_front[0])
ts.transform.rotation = Quaternion(*transform_right_to_front[1])
ts.header.stamp = rospy.Time.now()
transformed_cloud = do_transform_cloud(pc, ts)


print("Done")
rospy.loginfo(transformed_cloud)