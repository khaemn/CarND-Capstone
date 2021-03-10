#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import sys
import tf
import cv2
import yaml
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.base_waypoints = None
        self.base_wpt_tree = None
        self.base_wpt_2d = None
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
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.lights_coord_tree = None
        
        self.dataset_img_counter = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, lane):
        # Implemented based on the lessons
        self.base_waypoints = lane.waypoints
        if not self.base_wpt_2d:
            self.base_wpt_2d = \
                [[ waypoint.pose.pose.position.x, waypoint.pose.pose.position.y ] \
                                    for waypoint in self.base_waypoints ]
            self.base_wpt_tree = KDTree(self.base_wpt_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        # As there is usually only few traffic light in sight, it is effective
        # enough to re-generate a KDTree for them, in order to solve the
        # closest traffic light finding problem.
        lights_xy_coords = \
            [[ light.pose.pose.position.x, light.pose.pose.position.y ] \
                                        for light in self.lights ]
        self.lights_coord_tree = KDTree(lights_xy_coords)

    def distance_in_wpts_along_track(self, idx1, idx2):
        # This function assumes that the track is cyclic and the waypoints
        # indexes grow along the track (except the cycle merging point)
        if idx1 <= idx2:
            return idx2 - idx1
        return idx2 + len(self.base_waypoints) - idx1

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
        rospy.logerr("IMAGE received!  wpt idx  %d, lightstate  %r", light_wp, state)

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

    def get_closest_waypoint(self, x,y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            x,y: position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        if not self.base_wpt_tree:
            return 0
            
        closest_idx = self.base_wpt_tree.query([x, y], 1)[1]

        return closest_idx

    def clamp_saturation(self, img):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        sat = hsv[:,:,1]
        val = hsv[:,:,2]
        mask = np.zeros(sat.shape)
        mask[sat > 180] += 100
        mask[val > 150] += 100
        mask[mask < 200] = 0
        mask[mask >= 200] = 1
        return mask.astype(np.uint8)
        
    def detect_state(self, masked_light):
        red_mean = masked_light[:,:,2].mean()
        green_mean = masked_light[:,:,1].mean()
        blue_mean = masked_light[:,:,0].mean()
        red_green_diff_ratio = 1 / (abs(red_mean - green_mean) + 0.01)
        non_blue_ratio = (red_mean + green_mean) / 2 * blue_mean
        yellow_mean = red_green_diff_ratio * non_blue_ratio
        # States: 0 - red, 1 - yellow, 2 - green, 4 - Unknown
        indices = np.array([red_mean, yellow_mean, green_mean])
        if (indices.max() < 0.1):
            return TrafficLight.UNKNOWN
        return indices.argmax()
        
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

        #cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        mask = self.clamp_saturation(cv_image)
        masked = np.zeros(cv_image.shape)
        masked = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        state = self.detect_state(masked)
        #cv2.imwrite("/home/rattus/Free/Udacity/CarND-Capstone/imgs/traffic/{:05d}-{:01d}-{:01d}.jpg"
        #                .format(self.dataset_img_counter, state, light.state),
        #            cv_image)
        self.dataset_img_counter += 1

        # TODO: use classifier instead of the simulator data
        return state # light.state
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        if not self.base_waypoints:
            return -1, TrafficLight.UNKNOWN
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position_wpt_idx = self.get_closest_waypoint(self.pose.pose.position.x,
                                                             self.pose.pose.position.y)

        closest_light_number = -1
        closest_light_wpt_distance = len(self.base_waypoints) + 1
        for i, light in enumerate(self.lights):
            light_wpt = self.get_closest_waypoint(light.pose.pose.position.x,
                                                  light.pose.pose.position.y)
            distance = self.distance_in_wpts_along_track(car_position_wpt_idx, light_wpt)
            if (distance < closest_light_wpt_distance):
                closest_light_number = i
                closest_light_wpt_distance = distance
        stopline_coords = stop_line_positions[closest_light_number]
        stopline_wpt_idx = self.get_closest_waypoint(stopline_coords[0], stopline_coords[1])
        light = self.lights[closest_light_number]

        if light:
            light_state = self.get_light_state(light)
            return stopline_wpt_idx, light_state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
