#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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

DECELERATION = 2.0 # Absolute value of planned deceleration in m/s^2

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)
        
        #store the max velocity, already converted from km/h to m/s
        self.c_max_velocity = rospy.get_param('waypoint_loader/velocirty', 40.) / 3.6

        # Add other member variables
        self.waypoints_ref = None
        self.cur_wp_ref_idx = 0
        
        self.traffic_wp_idx = -1
        self.waypoints_with_reduced_velocity = []

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
            #avoid dump otherwise we'll be flooded in site/launch
#             rospy.logwarn('WaypointUpdater received pose data but reference waypoints are not initialized yet')
            pass
        else:
            # Log status of incoming data
            rospy.loginfo('WaypointUpdater rec: pose data (%.2f, %.2f, %.2f)', msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            # Calculate cur_wp_ref_idx
            min_dist = 100000.0
            min_idx  = self.cur_wp_ref_idx
            #remember the waypoint we've send out before to avoid
            #unnecessary updates
            prev_Last_wp_index = self.cur_wp_ref_idx
            start_idx = self.cur_wp_ref_idx - 2
            if (start_idx < 0):
                start_idx = start_idx + len(self.waypoints_ref.waypoints)
            for i in range(start_idx, start_idx + len(self.waypoints_ref.waypoints)):
                idx = i % len(self.waypoints_ref.waypoints)
                cur_dist = self.dist_3d(msg.pose.position, self.waypoints_ref.waypoints[idx].pose.pose.position)
                if (cur_dist < min_dist):
                    min_dist = cur_dist
                    min_idx  = idx
                if (min_dist < 5 and cur_dist > 10 * min_dist):
                    break
            dx = self.waypoints_ref.waypoints[min_idx].pose.pose.position.x - msg.pose.position.x
            dy = self.waypoints_ref.waypoints[min_idx].pose.pose.position.y - msg.pose.position.y
            heading = np.arctan2(dy, dx)
            (roll, pitch, yaw) = self.get_roll_pitch_yaw(msg.pose.orientation)
            angle = np.abs(yaw - heading)
            angle = np.minimum(angle, 2.0 * np.pi - angle)
            if (angle > np.pi / 4.0):
                self.cur_wp_ref_idx = (min_idx + 1) % len(self.waypoints_ref.waypoints)
            else:
                self.cur_wp_ref_idx = min_idx
            # Calculate self.waypoints_out
            #ONLY if the waypoint really has changed
            if prev_Last_wp_index != self.cur_wp_ref_idx:
              #send an update on the waypoint list
              self.filter_and_send_waypoints()
              wp = self.waypoints_ref.waypoints[self.cur_wp_ref_idx]
              waypoint_pos = wp.pose.pose.position
              waypoint_speed = wp.twist.twist.linear.x
              rospy.loginfo('WaypointUpdater pub: from index %i: (%.2f, %.2f, %.2f) with speed %.2f...'\
                            , self.cur_wp_ref_idx, waypoint_pos.x, waypoint_pos.y, waypoint_pos.z, waypoint_speed)
        pass
      
    #Copy the waypoints from the current car waypoints up to 
    #LOOKAHEAD_WPS waypoints in total and send them out  
    def filter_and_send_waypoints(self):
      if None == self.waypoints_ref:
        return
      rWaypoints = Lane()
      pos = self.cur_wp_ref_idx
      wp = self.waypoints_ref.waypoints
      rWaypoints.header = self.waypoints_ref.header
      rWaypoints.waypoints = wp[pos: min(pos+LOOKAHEAD_WPS, len(wp))]
      size = len(rWaypoints.waypoints)
      if size < LOOKAHEAD_WPS:
        rWaypoints.waypoints += wp[:LOOKAHEAD_WPS-size]
      self.final_waypoints_pub.publish(rWaypoints)
      

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
        #make sure, that none of the waypoints violates the max-speed condition
        counter = 0
        for wp in self.waypoints_ref.waypoints:
          if self.get_waypoint_velocity(wp) > self.c_max_velocity:
            wp.twist.twist.linear.x = self.c_max_velocity
            counter += 1
        rospy.loginfo('WaypointUpdater is initialized with {0} reference waypoints'\
                      ' - total of {1} were adjusted in velocity'\
                      .format(len(self.waypoints_ref.waypoints), counter))
        pass
        
    # Callback to receive topic /traffic_waypoint
    # (index of waypoint where to stop in front of the next red traffic light)
    #       data  Int32
    def traffic_cb(self, msg):
        # Log status of incoming data
        rospy.loginfo('WaypointUpdater rec: traffic waypoint index %i', msg.data)
        self.no_traffic_count = 0
        self.traffic_wp_idx = msg.data
        self.calc_waypoints_out()
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
        
    def calc_waypoints_out(self):
#         # Calculate the unconstrained case (no traffic light / obstacle)
        if None == self.waypoints_ref:
          return    
        #TODO: move to the constant section
        DISTANCE_SECURITY_ZERO_SPEED = 2.
        DISTANCE_SECURITY_ONE_SPEED = 3.
        
        
        #use reference to avoid long names
        all_wp = self.waypoints_ref.waypoints
        
        # Consider traffic
        # only if the waypoints_with_reduced_velocity isn't None, we need
        # to recalculate the velocity of the waypoints
        if (self.traffic_wp_idx != -1) and (0 == len(self.waypoints_with_reduced_velocity)):
            # Check if traffic light is in planning range
            light_dist = self.distance(self.cur_wp_ref_idx, self.traffic_wp_idx)
            
            # TODO: consider maximum comfortable jerk by choosing smooth velocity curve
            # Determine required deceleration
            dec        = DECELERATION
            cur_speed  = self.get_waypoint_velocity(all_wp[self.cur_wp_ref_idx])
            dec_time   = (cur_speed - 1.) / dec
            dec_dist   = 0.5 * dec * dec_time * dec_time
            light_dist = self.distance(self.cur_wp_ref_idx, self.traffic_wp_idx)
            light_dist_plus_security = light_dist + DISTANCE_SECURITY_ONE_SPEED
            if (light_dist_plus_security < dec_dist):
                dec = 0.5 * cur_speed * cur_speed / dec_dist
                rospy.logwarn('WaypointUpdater needs to plan uncomfortable deceleration %.2f m/s^2', dec)
            else:
                rospy.loginfo('WaypointUpdater plans comfortable deceleration %.2f m/s^2', dec)
                
            
            # Adjust speed
            distance_wp_tl = 0.

            current_wp_idx = self.traffic_wp_idx
            prev_wp_idx = self.traffic_wp_idx
            while distance_wp_tl < dec_dist:
              current_wp_speed = self.get_waypoint_velocity(all_wp[current_wp_idx])
              self.waypoints_with_reduced_velocity.append( [current_wp_idx, current_wp_speed])
              distance_wp_tl = self.dist_3d(all_wp[self.traffic_wp_idx].pose.pose.position, all_wp[prev_wp_idx].pose.pose.position)
              wp_speed = 0.
              if distance_wp_tl < DISTANCE_SECURITY_ZERO_SPEED:
                wp_speed = 0.
              elif distance_wp_tl < DISTANCE_SECURITY_ONE_SPEED:
                wp_speed = 1.
              else :
                #we must done one step already
                assert(prev_wp_idx != current_wp_idx)
                wp_time = math.sqrt(2. * distance_wp_tl / dec)
                wp_speed = min( (1. + wp_time * dec)\
                                , self.c_max_velocity)
              self.set_waypoint_velocity(all_wp, current_wp_idx, wp_speed)
              prev_wp_idx = current_wp_idx
              #iterate from the traffic light back to current car position
              current_wp_idx = self.prev_waypoint(current_wp_idx)
        elif (self.traffic_wp_idx == -1):
          #restore the original speed
          if 0 != len(self.waypoints_with_reduced_velocity):
            for entry in self.waypoints_with_reduced_velocity:
              self.set_waypoint_velocity(all_wp, entry[0], entry[1])
            self.waypoints_with_reduced_velocity = []
            #send an update
            self.filter_and_send_waypoints()
            
        return

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, wp_idx, velocity):
        waypoints[wp_idx].twist.twist.linear.x = velocity
        pass
        
    def next_waypoint(self, wp_idx):
      if self.waypoints_ref is not None:
        return (wp_idx+1) % len(self.waypoints_ref.waypoints)
      return wp_idx
    def prev_waypoint(self, wp_idx):
      if self.waypoints_ref is not None:
        return (wp_idx-1) % len(self.waypoints_ref.waypoints)
      return wp_idx

    def dist_3d(self, a, b):
        return math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

    def distance(self, wp_idx_first, wp_idx_last):
        if None == self.waypoints_ref:
          return 0    
        dist = 0
        wp_idx_distance = 0
        #consider overflows in waypoint list as well
        all_wp_length = len(self.waypoints_ref.waypoints)
        if(wp_idx_first < wp_idx_last):
          wp_idx_distance = wp_idx_last-wp_idx_first
        else:
          wp_idx_distance = all_wp_length - wp_idx_first + wp_idx_last
        for i in range(wp_idx_first, (wp_idx_first+wp_idx_distance)):
            idx = i % all_wp_length
            next_idx = (idx + 1) % all_wp_length
            dist += self.dist_3d(self.waypoints_ref.waypoints[idx].pose.pose.position, self.waypoints_ref.waypoints[next_idx].pose.pose.position)
        return dist

    def get_roll_pitch_yaw(self, ros_quaternion):
        orientation_list = [ros_quaternion.x, ros_quaternion.y, ros_quaternion.z, ros_quaternion.w]
        return euler_from_quaternion(orientation_list) # returns (roll, pitch, yaw)
        
    def get_ros_quaternion(roll, pitch, yaw):
        return Quaternion(*quaternion_from_euler(roll, pitch, yaw)) # returns Quaternion
        

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
