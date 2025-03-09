#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import csv


global_costmap = None

def costmap_callback(data):
    global global_costmap
    global_costmap = data

def save_costmap_to_csv(costmap, filename="costmap.csv"):
    width = costmap.info.width
    height = costmap.info.height
    resolution = costmap.info.resolution
    origin_x = costmap.info.origin.position.x
    origin_y = costmap.info.origin.position.y
    

    grid = np.array(costmap.data).reshape((height, width))
    
 
    with open(filename, mode='w') as file:
        writer = csv.writer(file)

        writer.writerow(['Costmap Info:', 'Width', width, 'Height', height, 'Resolution', resolution, 'Origin X', origin_x, 'Origin Y', origin_y])
        for row in grid:
            writer.writerow(row)
    rospy.loginfo("Costmap saved to %s", filename)

def main():
    rospy.init_node('costmap_saver', anonymous=True)
    rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, costmap_callback)
    
    rate = rospy.Rate(1) # 1 Hz
    rospy.loginfo("Waiting for the costmap...")
    
    while not rospy.is_shutdown():
        if global_costmap is not None:
            save_costmap_to_csv(global_costmap)
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass