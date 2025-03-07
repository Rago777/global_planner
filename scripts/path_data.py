#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
import csv
import codecs
import math

def path_callback(data):
    if not data.poses:
        rospy.logwarn("Received an empty path.")
        return

    file_name = 'global_path_data.csv'
    rospy.loginfo("Saving global path to %s", file_name)

    with codecs.open(file_name, 'w', encoding='utf-8') as csvfile:
        fieldnames = ['index', 'x', 'y', 'yaw']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for idx, pose_stamped in enumerate(data.poses):
            orientation = pose_stamped.pose.orientation
            yaw = math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
            
            writer.writerow({
                'index': idx,
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'yaw': yaw
            })
    
    rospy.loginfo("Global path saved successfully to %s", file_name)
    rospy.loginfo("Total number of path points: %d", len(data.poses))
    rospy.signal_shutdown("Global path saved, shutting down node.")

def capture_global_path():
    rospy.init_node("capture_global_path", anonymous=True)
    
    topic_name = rospy.get_param('~global_path_topic', "/move_base/GlobalPlanner/plan")
    rospy.Subscriber(topic_name, Path, path_callback, queue_size=10)
    
    rospy.loginfo("Capturing global path published on '%s'.", topic_name)
    rospy.spin()

if __name__ == '__main__':
    try:
        capture_global_path()
    except rospy.ROSInterruptException:
        pass
