#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math
STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        self.waypoints_flag = False

        while self.waypoints_flag == False: #rkb nneds to wait for waypount load otherwise it doesn't load why ???
            rospy.sleep(1/100.0)
            #pass

        self.light_classifier = TLClassifier()
        rospy.loginfo("light_classifier")
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.has_image = False
        self.traffic_lt_flag = False
        #self.waypoints_flag = False

        self.publish_flag = False

        #need to uncomment if we don't want to use classifier and rely on grouund truth,
        #needs to implement locking mechanism if we want both feature
        self.camera_transition = False
        self.traffic_light_pub() #for test to debug without camera, needs to be disabled

        rospy.spin()

    def traffic_light_pub(self):
        #this is for testing and the call needs to be disabled

        rate = rospy.Rate(50) # 50Hz
        #rospy.loginfo("Is shutdown: %d", rospy.is_shutdown())
        while not rospy.is_shutdown():

            if self.publish_flag == True and self.has_image == False :
                if (self.traffic_lt_flag == True) and (self.waypoints_flag == True):
                    self.camera_transition = False

                    #rospy.loginfo("Publish from non camera mode where it will publish grnd_state")
                    self.publish_light() #for debug and made it member of the class from prev version

                    self.camera_transition = True #now camera_cb can take care of publising if it is on

                    self.publish_flag = False
            rate.sleep()
            #rospy.loginfo("Is shutdown: %d %d %d %d %d", rospy.is_shutdown(), self.publish_flag, self.has_image, self.waypoints_flag,  self.traffic_lt_flag)


    def pose_cb(self, msg):
        self.pose = msg
        self.publish_flag = True
        #rospy.loginfo( "pose_cb::::pose_x:%d, pose_y:%d", self.pose.pose.position.x, self.pose.pose.position.y)
        #if (self.traffic_lt_flag == True) and (self.waypoints_flag == True):
        #    publish_light(self) #for debug

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_flag = True

    def traffic_cb(self, msg):
        self.lights = msg.lights

        #for idx, light in enumerate(self.lights):
        #    rospy.loginfo( "traffic_cb:::: %d pose_x:%f, pose_y:%f", idx, light.pose.pose.position.x, light.pose.pose.position.y)
        self.traffic_lt_flag = True

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg

        #rospy.loginfo("Is shutdown: %d %d %d %d %d", rospy.is_shutdown(), self.publish_flag, self.has_image, self.waypoints_flag,  self.traffic_lt_flag)
        if (self.traffic_lt_flag == True) and (self.waypoints_flag == True) and (self.camera_transition == True): #ensures safe state to transition

            #rospy.loginfo("Publish from camera callback")
            light_wp, state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        #return 0
        pose_x = pose.position.x
        pose_y = pose.position.y
        pose_z = pose.position.z

        #rospy.loginfo( "pose_x:%d, pose_y:%d", pose_x, pose_y)

        if self.waypoints != None:
            total_wps_ = len(self.waypoints.waypoints)
        else:
            return None

        min_idx= 0
        min_distance = 0x7fffffff #very high value
        start_idx = 0
        end_idx = total_wps_
        wps = self.waypoints.waypoints
        for idx, wp in enumerate(wps[start_idx:end_idx]):

            x_dist = wp.pose.pose.position.x - pose_x
            y_dist = wp.pose.pose.position.y - pose_y
            z_dist = wp.pose.pose.position.z - pose_z

            dist_from_pose = math.sqrt( x_dist * x_dist + y_dist * y_dist + z_dist * z_dist )

            if dist_from_pose < min_distance: #should we <=
                min_distance = dist_from_pose
                min_idx = idx

        #rospy.loginfo( "closest_idx:%d, total_wps_:%d", min_idx, total_wps_)
        closest_idx = min_idx
        return closest_idx



    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #rospy.loginfo("classifier: %d", self.light_classifier.init_flag)
        if self.light_classifier.init_flag == False: # why.. since loading graph takes time and we need before publlishing
            #rospy.loginfo("classifier: %d", self.light_classifier.init_flag)
            return -1, TrafficLight.UNKNOWN


        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        #if(self.pose) or 1:
        car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)
        if car_position != None:
            #rospy.loginfo("car pos: %d ", car_position)
            light, light_wp = self.get_closest_light(car_position, stop_line_positions)
            #rospy.loginfo( light, light_wp)


        if light:

            light_heading = light_wp - car_position

            #start publishing if ahead stop is 150 waypoint far
            if light_heading < 200  and light_heading > 0:
                state = self.get_light_state(light)

                ## all the lights in the simulator has
                # same state so just take  the first one or otherwise
                #we need to locate the corresponding light message as done in get_light_ground_state
                ground_state = self.get_light_ground_state(light, self.lights) #self.lights[0].state #for debug sending ground truth

                if self.has_image == False:
                    state = ground_state

                #rospy.loginfo("Light state: (%d  ->%d) %d", ground_state, self.lights[0].state, state)

                return light_wp-3, state
            else:
                return -1, TrafficLight.UNKNOWN

        #self.waypoints = None
        #rospy.loginfo("NONE")
        return -1, TrafficLight.UNKNOWN

    def get_closest_light(self, car_position, stop_line_positions): #added by rkb

        closest_light = None
        distance = 0x7fffffff

        car_pose = self.waypoints.waypoints[car_position].pose
        for light in stop_line_positions:

            x_dist = car_pose.pose.position.x -  light[0]
            y_dist = car_pose.pose.position.y -  light[1]

            dist = math.sqrt(x_dist*x_dist + y_dist * y_dist)
            if dist < distance:# and (x_dist < 0):
                closest_light = light
                distance = dist

            #rospy.loginfo("light_wp: %f %f %f %f state:%d %d", dist, distance, y_dist, x_dist, state, car_position)

        #rospy.loginfo("close light: pose_x: %f, pose_y:%f", closest_light[0], closest_light[1])
        close_wp_idx = -1
        if   closest_light != None:
            #closest_light.pose.pose.position.z = 0
            close_wp_idx = self.get_closest_waypoint_for_lt(closest_light)
            #check whether the light wp is ahead of car postion
            light_heading = close_wp_idx - car_position

            if light_heading < 0:
                close_wp_idx = -1
                closest_light = None

        return closest_light, close_wp_idx

    def get_closest_waypoint_for_lt(self, light):

        pose_x = light[0]
        pose_y = light[1]
        pose_z = 0

        if self.waypoints != None:
            total_wps_ = len(self.waypoints.waypoints)
        else:
            return None

        min_idx= 0
        min_distance = 0x7fffffff #very high value
        start_idx = 0
        end_idx = total_wps_
        wps = self.waypoints.waypoints
        for idx, wp in enumerate(wps[start_idx:end_idx]):

            x_dist = wp.pose.pose.position.x - pose_x
            y_dist = wp.pose.pose.position.y - pose_y
            z_dist = 0

            dist_from_pose = ( x_dist * x_dist + y_dist * y_dist + z_dist * z_dist )

            if dist_from_pose < min_distance: #should we <=
                min_distance = dist_from_pose
                min_idx = idx

        closest_idx = min_idx
        return closest_idx


    def get_light_ground_state(self, stop_point, light_msg):

        if light_msg == None:
            TrafficLight.UNKNOWN

        state =  light_msg[0].state
        min_distance = 0x7fffffff
        closest_lt_from_stop = None

        for lt in light_msg:
            x_dist = stop_point[0] - lt.pose.pose.position.x
            y_dist = stop_point[1] - lt.pose.pose.position.y
            dist = math.sqrt(x_dist * x_dist + y_dist * y_dist)
            if dist < min_distance:
                closest_lt_from_stop = lt
                min_distance = dist

        if closest_lt_from_stop != None:
            return closest_lt_from_stop.state
        else:
            TrafficLight.UNKNOWN

    def publish_light(self): #debug

        #rospy.loginfo(" publish_light ")
        light_wp, state = self.process_traffic_lights()
        #rospy.loginfo("light_wp: %f state: %d", light_wp, state)

        #if state == TrafficLight.RED:
        #    self.upcoming_red_light_pub.publish(Int32(light_wp))

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if 1:
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1



if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
