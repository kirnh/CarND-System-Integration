#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import math
import copy

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
#LOOKAHEAD_WPS = 10

call_cnt = 0

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #subscribe to /traffic_waypoint topic
        msg_type = Int32 #to change appropriate msg type
        rospy.Subscriber('/traffic_waypoint', msg_type, self.traffic_cb)


        #msg_type = 'None' #to change appropriate msg type
        #rospy.Subscriber('/obstacle_waypoint', msg_type, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints_list = None
        self.master_waypoints_list = None

        self.curr_pose_x = 0
        self.curr_pose_y = 0
        self.curr_pose_z = 0
        self.curr_yaw_orientation = 0
        self.curr_pose_header = None
        self.start = 0
        self.publish_flag = False
        self.traffic_light_wp_idx = None
        self.current_pose_wp_idx = 0
        self.way_pts_load_flag = None

        #self.traffic_light_wp_idx = 300
        self.final_waypoint_pub()

        rospy.spin()

    def final_waypoint_pub(self):

        rate = rospy.Rate(50) # 50Hz

        while not rospy.is_shutdown():

            if self.publish_flag == True and self.way_pts_load_flag == True:
                get_final_waypoints(self)

            rate.sleep()



    def pose_cb(self, msg):
        # TODO: Implement

        #self.curr_pose_header = msg.header
        self.curr_pose_x = msg.pose.position.x
        self.curr_pose_y = msg.pose.position.y
        self.curr_pose_z = msg.pose.position.z
        self.curr_yaw_orientation = msg.pose.orientation.z #only concerned with z orientation i.e yaw

        self.publish_flag = True #need to change it as true rkb

        #print curr_postion
        #pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.master_waypoints_list  = waypoints.waypoints[:]
        self.waypoints_list = copy.deepcopy(self.master_waypoints_list)
        self.way_pts_load_flag = True
        #self.publish_flag = True #nned to comment rkb
        #pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        self.traffic_light_wp_idx = msg.data
        #rospy.loginfo("traffic_light_wp_idx : %d", self.traffic_light_wp_idx)




    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
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

    def publish(self, waypoints):
        lane = Lane()
        lane.header.frame_id = '/trajectory'
        lane.header.stamp = rospy.Time(0)
        lane.waypoints = waypoints
        self.final_waypoints_pub.publish(lane)



def get_final_waypoints(waypoint_update):
    #publish next  waypoints on the

    #get the closest waypoint index

    total_wps_ = len(waypoint_update.waypoints_list)
    closest_idx = get_closest_wp(waypoint_update)
    waypoint_update.current_pose_wp_idx = closest_idx

    #rospy.loginfo("call: %d closest_idx:%f", call_cnt, closest_idx)

    final_waypoints = []


    for wp_cnt in range(0, LOOKAHEAD_WPS+1):

        wp = waypoint_update.waypoints_list[((closest_idx + 1) + wp_cnt) % total_wps_]
        curr_indx = ((closest_idx + 1) + wp_cnt) % total_wps_
        # set the velocity if there is an obstacle or red light is detected
        #velocity_kmph = 40
        #target_vel = (velocity_kmph * 1000.) / (60. * 60.)
        #waypoint_update.set_waypoint_velocity(waypoint_update.waypoints_list, curr_indx, target_vel)
        target_vel = waypoint_update.get_waypoint_velocity(waypoint_update.master_waypoints_list[curr_indx])
        target_vel_next = waypoint_update.get_waypoint_velocity(waypoint_update.master_waypoints_list[(curr_indx + 1) % total_wps_])

        if target_vel == 0 and target_vel_next > 0:
            target_vel = target_vel_next * 3.0 / 4 # to solve end of trajectory problem where the loader set the value as zero

        waypoint_update.set_waypoint_velocity(waypoint_update.waypoints_list, curr_indx, target_vel)

        final_waypoints.append(wp)


    closest_wp_idx = closest_idx #get_closest_wp(waypoint_update)

    #if waypoint_update.traffic_light_wp_idx != None:
    #    rospy.loginfo("curr_wp_idx :: %f traffic %f", closest_wp_idx, waypoint_update.traffic_light_wp_idx)

    if waypoint_update.traffic_light_wp_idx != None and \
        (waypoint_update.traffic_light_wp_idx > 0):

        wp_to_stop = waypoint_update.traffic_light_wp_idx
        tf_distance_from_wp = wp_to_stop - closest_wp_idx


        if tf_distance_from_wp > 0  and tf_distance_from_wp < 30:
            #rospy.loginfo("Update velocity")
            #waypoint_update.current_pose_wp_idx = closest_wp_idx
            update_velocity(waypoint_update, closest_wp_idx, wp_to_stop)

    #for idx, final_wp in enumerate(final_waypoints):
    #    vel = waypoint_update.get_waypoint_velocity(final_wp)
    #    rospy.loginfo("final vel pub curr_indx: %d val %f", idx, vel)

    #publish
    waypoint_update.publish(final_waypoints)
    waypoint_update.publish_flag = False


def get_closest_wp(waypoint_update):
        total_wps_ = len(waypoint_update.waypoints_list)

        min_idx= waypoint_update.start
        min_distance = 0xffffffff #very high value
        start_idx = waypoint_update.start
        end_idx = total_wps_
        for idx, wp in enumerate(waypoint_update.waypoints_list[start_idx:end_idx]):

            x_dist = wp.pose.pose.position.x - waypoint_update.curr_pose_x
            y_dist = wp.pose.pose.position.y - waypoint_update.curr_pose_y
            z_dist = wp.pose.pose.position.z - waypoint_update.curr_pose_z

            dist_from_pose = math.sqrt( x_dist * x_dist + y_dist * y_dist + z_dist * z_dist )

            if dist_from_pose < min_distance: #should we <=
                min_distance = dist_from_pose
                min_idx = waypoint_update.start + idx

        closest_idx = min_idx
        return closest_idx


def update_velocity(wps_update, curr_pose_wp_idx,traffic_wp_idx):

        curr_vel = wps_update.get_waypoint_velocity(wps_update.waypoints_list[curr_pose_wp_idx])
        step = 0
        total_distance = wps_update.distance(wps_update.waypoints_list,
                                      curr_pose_wp_idx,
                                      wps_update.traffic_light_wp_idx)
        reduced_wp_vel = curr_vel

        for wp_cnt in range(curr_pose_wp_idx, traffic_wp_idx+1):

            wp = wps_update.waypoints_list[wp_cnt]
            curr_indx = wp_cnt

            # set the velocity if there is an obstacle or red light is detected

            wp_vel = wps_update.get_waypoint_velocity(wp)

            dist_step =  wps_update.distance(wps_update.waypoints_list,
                                      curr_indx,
                                      curr_indx + 1)

            step = curr_vel * dist_step / total_distance
            reduced_wp_vel = reduced_wp_vel - step
            #reduced_wp_vel = 0
            #rospy.loginfo("curr_indx: %d red_val%f, step:%f  %f %f", curr_indx, reduced_wp_vel,
            #                step, total_distance, dist_step)

            if reduced_wp_vel < 0.1:
                reduced_wp_vel = 0
            #reduced_wp_vel = max(0, reduced_wp_vel)
            wps_update.set_waypoint_velocity(wps_update.waypoints_list, curr_indx + 1, reduced_wp_vel)

            #rospy.loginfo("curr_indx: %d get_red_val%f", curr_indx, wps_update.get_waypoint_velocity(wp))


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
