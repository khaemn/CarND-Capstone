from styx_msgs.msg import TrafficLight
import cv2
import numpy as np

class TLClassifier(object):
    def __init__(self):
        self.saturation_threshold = 160
        self.value_threshold = 200
        self.matching_score = 50
        self.color_majority_level = 0.15
        pass

    def brightly_colored_objects_mask(self, img):
        ''' Expects a color image in BGR colorspace
        Returns a binary mask around traffic light circles,
        supposing the traffic lights are the most saturated objects '''
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        sat = hsv[:,:,1]
        val = hsv[:,:,2]
        mask = np.zeros(sat.shape, dtype=np.uint8)
        # Any pixel with high _saturation_ is probably a
        # intensively colored object, which means it is higly probably unnatural
        # e.g. not a sky, not a ground, etc.
        mask[sat >= self.saturation_threshold] += self.matching_score
        # Any pixel with a high _value_ is probably from a bright object, e.g.
        # high-luminous object - a light, reflection, etc.
        mask[val >= self.value_threshold] += self.matching_score
        # Any pixel that has a high saturation AND high value is probably from
        # an intensively colored luminous object, and there is a decent chance
        # the only bright intensively color object above the road is a traffic light.
        mask[mask < self.matching_score * 2] = 0
        mask[mask >= self.matching_score * 2] = 1
        return mask
        
    def cut_traffic_light_background(self, img, mask):
        masked = np.zeros(img.shape, dtype=img.dtype)
        masked = cv2.bitwise_and(img, img, mask=mask)
        return masked
        
    def detect_state(self, masked_light):
        ''' Expects a color BGR image, where only (or at least mostly)
        traffic light circles are left on a black background 
        Returns a red, yellow, green state if there is majority of
        non-black pixels of that colour, otherwise returns Unknown'''
        red_mean = masked_light[:,:,2].mean()
        green_mean = masked_light[:,:,1].mean()
        blue_mean = masked_light[:,:,0].mean()

        # A "yellow" pixel is a pixel that is not "blue" and the one
        # with close values of red and green.
        red_green_diff_ratio = (red_mean + green_mean) / (abs(red_mean - green_mean)*2. + 0.01)
        non_blue_ratio = max(0., (red_mean + green_mean) - (2. * blue_mean))
        yellow_mean = red_green_diff_ratio * non_blue_ratio

        # States: 0 - red, 1 - yellow, 2 - green, 4 - Unknown
        indices = np.array([red_mean, yellow_mean, green_mean])

        if (indices.max() < self.color_majority_level):
            return TrafficLight.UNKNOWN

        return indices.argmax()


    def get_classification(self, cv_image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        mask = self.brightly_colored_objects_mask(cv_image)
        masked = self.cut_traffic_light_background(cv_image, mask)
        state = self.detect_state(masked)
        
        return state
