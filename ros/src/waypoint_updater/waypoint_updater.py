#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Waypoint, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Waypoint, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_ref = None
        self.cur_wp_ref_idx = 0
        self.waypoints_out = None

        rospy.spin()

    # Callback to receive topic /current_pose
    # msg   a Pose with reference coordinate frame and timestamp
    #       msg.header Header
    #       msg.pose   Pose
    # Header:
    #   'sequence ID: consecutively increasing ID'
    #     uint32 seq
    #   'Two-integer timestamp that is expressed as:
    #   * stamp.sec: seconds (stamp_secs) since epoch (in Python this is called 'secs')
    #   * stamp.nsec: nanoseconds since stamp_secs (in Python this is called 'nsecs')'
    #     time stamp
    #   'Frame this data is associated with (0: no frame, 1: global frame)'
    #   string frame_id
    # Pose:
    #   'contains the position of a point in free space'
    #     Point position
    #       float64 x
    #       float64 y
    #       float64 z
    #   'represents an orientation in free space in quaternion form'
    #     Quaternion orientation
    #       float64 x
    #       float64 y
    #       float64 z
    #       float64 w
    def pose_cb(self, msg):
		if self.waypoints_ref == None:
			# Log warning of incoming data
			rospy.logwarn('WaypointUpdater received pose data but reference waypoints are not initialized yet')
		else:
			# Log status of incoming data
			rospy.loginfo('WaypointUpdater received pose data (%f, %f, %f)', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
			# Calculate cur_wp_ref_idx
			min_dist = 100000.
			min_idx  = self.cur_wp_ref_idx
			for i in range(self.cur_wp_ref_idx, self.cur_wp_ref_idx + len(self.waypoints_ref.waypoints)):
				idx = i % len(self.waypoints_ref.waypoints)
				cur_dist = self.dist_3d(msg.pose.position, self.waypoints_ref.waypoints[idx].pose.pose.position)
				if cur_dist < min_dist:
					min_dist = cur_dist
					min_idx  = idx
			dx = self.waypoints_ref.waypoints[min_idx].pose.pose.position.x - msg.pose.position.x
			dy = self.waypoints_ref.waypoints[min_idx].pose.pose.position.y - msg.pose.position.y
			heading = np.arctan2(dy, dx)
			(roll, pitch, yaw) = self.get_roll_pitch_yaw(msg.pose.orientation)
			angle = np.abs(yaw - heading)
			angle = np.minimum(angle, 2.0 * np.pi - angle)
			if angle > np.pi / 4.0:
				self.cur_wp_ref_idx = (min_idx + 1) % len(self.waypoints_ref.waypoints)
			else:
				self.cur_wp_ref_idx = min_idx
			# Calculate self.waypoints_out
			self.waypoints_out.header = self.waypoints_ref.header
			self.waypoints_out.waypoints = []
			for i in range(self.cur_wp_ref_idx, self.cur_wp_ref_idx + LOOKAHEAD_WPS):
				idx = len(self.waypoints_ref.waypoints) % i
				self.waypoints_out.waypoints.append(self.waypoints_ref.waypoints[idx])
			# Publish the data
			rospy.loginfo('WaypointUpdater sends waypoint data starting from index %i: (%f, %f, %f)...', self.cur_wp_ref_idx, self.waypoints_out.pose.position.x, self.waypoints_out.pose.position.y, self.waypoints_out.pose.position.z)
			self.final_waypoints_pub.publish(self.waypoints_out)
        pass

    # Callback to receive topic /base_waypoints
    #       msg.header    Header
    #       msg.waypoints Waypoint[]
    # Header:
    #   'sequence ID: consecutively increasing ID'
    #     uint32 seq
    #   'Two-integer timestamp that is expressed as:
    #   * stamp.sec: seconds (stamp_secs) since epoch (in Python this is called 'secs')
    #   * stamp.nsec: nanoseconds since stamp_secs (in Python this is called 'nsecs')'
    #     time stamp
    #   'Frame this data is associated with (0: no frame, 1: global frame)'
    #   string frame_id
    # Waypoint:
    #   PoseStamped pose
    #   TwistStamped twist
    # PoseStamped:
    #   Header header
    #     ...
    #   Pose pose
    #     'contains the position of a point in free space'
    #       Point position
    #         float64 x
    #         float64 y
    #         float64 z
    #     'represents an orientation in free space in quaternion form'
    #       Quaternion orientation
    #         float64 x
    #         float64 y
    #         float64 z
    #         float64 w
    # TwistStamped:
    #   Header header
    #     ...
    #   Twist twist
    #     'expresses velocity in free space linear parts'
    #       Vector3  linear
    #         float64 x
    #         float64 y
    #         float64 z
    #     'expresses velocity in free space angular parts'
    #       Vector3  angular
    #         float64 x
    #         float64 y
    #         float64 z
    def waypoints_cb(self, waypoints):
        # Store waypoint data for later usage
        self.waypoints_ref = waypoints
		rospy.loginfo('WaypointUpdater is initialized with %i reference waypoints', len(self.waypoints_ref))
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp_idx, velocity):
        waypoints[wp_idx].twist.twist.linear.x = velocity

    def dist_3d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def distance(self, waypoints, wp_idx_first, wp_idx_last):
        dist = 0
        for i in range(wp_idx_first, wp_idx_last):
            dist += dist_3d(waypoints[i].pose.pose.position, waypoints[i+1].pose.pose.position)
        return dist

	def get_roll_pitch_yaw(self, ros_quaternion):
		orientation_list = [ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w]
		return euler_from_quaternion (orientation_list) # returns (roll, pitch, yaw)
		
	def get_ros_quaternion(roll, pitch, yaw):
		return Quaternion(*quaternion_from_euler(roll, pitch, yaw)) # returns Quaternion
		

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
