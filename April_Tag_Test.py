#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray

def callback(data):
    if not data.detections:
        return
    
    # Assume the first detected tag is the one we're interested in
    tag = data.detections[0]
    tag_id = tag.id[0]  # Get the tag ID
    tag_pose = tag.pose.pose.pose  # The pose of the tag in the camera frame

    try:
        # Transform the tag pose from the camera frame to the robot base frame
        transform = tf_buffer.lookup_transform('ur3_base_frame', tag_pose.header.frame.append(1.0))
        transformed_pose = tf2_geometry_msgs.do_transform_pose(tag_pose, transform)

        # Adjust the pose to move 10 cm along the z-axis of the tag's coordinate frame
        offset_pose = PoseStamped()
        offset_pose.pose = transformed_pose.pose
        offset_pose.pose.position.z += 0.1  # Adding 10 cm to the z-axis

        # Example print or use this pose for motion planning with MoveIt
        print("Tag ID: ", tag_id)
        print("Offset Pose: ", offset_post.pose.position)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.loginfo(e)

if __name__ == '__main__':
    rospy.init_node('apriltag_localizer')
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber("/tag_detections", AprilTag, Transform, callback)
    rospy.spin()
