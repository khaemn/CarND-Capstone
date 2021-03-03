#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 #Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.traffic_wpt_idx = -1

        self.loop()
        
    def loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                self.publish_waypoints(self.get_closest_wpt_idx())
            rate.sleep()

    def get_closest_wpt_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        if not self.waypoint_tree:
            return 0
            
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # Check if the closest wpt is ahead or behind the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_idx = 0 if closest_idx <= 1 else closest_idx - 1
        prev_coord = self.waypoints_2d[prev_idx]
        
        centerline_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(centerline_vect - prev_vect, pos_vect - centerline_vect)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self, idx):
        lane = Lane()
        lane.header = self.base_waypoints.header
        closest_idx = idx
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        no_red_traffic_light_ahead = \
                self.traffic_wpt_idx < closest_idx or self.traffic_wpt_idx > farthest_idx
        if no_red_traffic_light_ahead:
            lane.waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]
        else:
            # Decelerate (naiive)
            for wpt in self.base_waypoints.waypoints[closest_idx : farthest_idx]:
                new_wpt = wpt
                new_wpt.twist.twist.linear.x = 0.
                lane.waypoints.append(new_wpt)
        self.final_waypoints_pub.publish(lane)
        # rospy.logwarn("Closest WPT: %d", closest_idx)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        # Implemented based on the lessons
        self.base_waypoints = lane
        if not self.waypoints_2d:
            self.waypoints_2d = [[ waypoint.pose.pose.position.x, waypoint.pose.pose.position.y ]
                                    for waypoint in lane.waypoints ]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        
    def traffic_cb(self, msg):
        self.traffic_wpt_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        print('obstacle_cb msg: ', msg)
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
