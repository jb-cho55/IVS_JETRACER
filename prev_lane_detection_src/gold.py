#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge

class JdOpencvLaneDetect(object):
    def __init__(self):
        self.curr_steering_angle = 90

    def get_lane(self, frame):
        lane_lines, lane_lines_image = self.detect_lane(frame)
        return lane_lines, lane_lines_image

    def get_steering_angle(self, img_lane, lane_lines):
        if len(lane_lines) == 0:
            return 0, None
        new_steering_angle = self.compute_steering_angle(img_lane, lane_lines)
        self.curr_steering_angle = self.stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))

        curr_heading_image = self.display_heading_line(img_lane, self.curr_steering_angle)

        return self.curr_steering_angle, curr_heading_image

    def get_curvature(self, frame, lane_lines):
        if len(lane_lines) == 0:
            return None
        return self.compute_curvature(frame, lane_lines)

    def detect_lane(self, frame):
        edges = self.detect_edges(frame)
        if edges is None or edges.size == 0:
            rospy.logerr('No edges detected.')
            return [], frame
        
        cropped_edges = self.region_of_interest(edges)
        if cropped_edges is None or cropped_edges.size == 0:
            rospy.logerr('No cropped edges.')
            return [], frame

        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)
        lane_lines_image = self.display_lines(frame, lane_lines)
    
        return lane_lines, lane_lines_image

    def detect_edges(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([40, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        edges = cv2.Canny(mask, 200, 400)
        return edges

    def region_of_interest(self, canny):
        height, width = canny.shape
        mask = np.zeros_like(canny)

        polygon = np.array([[
            (0, height * (1/2)),
            (width, height * (1/2)),
            (width, height),
            (0, height),
        ]], np.int32)
        cv2.fillPoly(mask, polygon, 255)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image

    def detect_line_segments(self, cropped_edges):
        rho = 1
        angle = np.pi / 180
        min_threshold = 10
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=15, maxLineGap=4)

        if line_segments is not None:
            for line_segment in line_segments:
                rospy.logdebug('Detected line_segment:')
                rospy.logdebug("%s of length %s" % (line_segment, self.length_of_line_segment(line_segment[0])))

        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []
        if line_segments is None or len(line_segments) == 0:
            rospy.loginfo('No line segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary
        
        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    rospy.loginfo('Skipping vertical line segment (slope=inf): %s' % line_segment)
                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        if slope < -0.75:
                            left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        if slope > 0.75:
                            right_fit.append((slope, intercept))

        if len(left_fit) > 0:
            left_fit_average = np.average(left_fit, axis=0)
            lane_lines.append(self.make_points(frame, left_fit_average))

        if len(right_fit) > 0:
            right_fit_average = np.average(right_fit, axis=0)
            lane_lines.append(self.make_points(frame, right_fit_average))

        rospy.logdebug('lane lines: %s' % lane_lines)

        return lane_lines

    

    def stabilize_steering_angle(self, curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
        if num_of_lane_lines == 2:
            max_angle_deviation = max_angle_deviation_two_lines
        else:
            max_angle_deviation = max_angle_deviation_one_lane
        
        angle_deviation = new_steering_angle - curr_steering_angle
        if abs(angle_deviation) > max_angle_deviation:
            stabilized_steering_angle = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized_steering_angle = new_steering_angle
        rospy.loginfo('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
        return stabilized_steering_angle

    def compute_curvature(self, frame, lane_lines):
        if len(lane_lines) < 2:
            return None

        left_fit = np.polyfit((lane_lines[0][0][1], lane_lines[0][0][3]), (lane_lines[0][0][0], lane_lines[0][0][2]), 2)
        right_fit = np.polyfit((lane_lines[1][0][1], lane_lines[1][0][3]), (lane_lines[1][0][0], lane_lines[1][0][2]), 2)

        y_eval = frame.shape[0]

        left_curverad = ((1 + (2*left_fit[0]*y_eval + left_fit[1])**2)**1.5) / np.absolute(2*left_fit[0])
        right_curverad = ((1 + (2*right_fit[0]*y_eval + right_fit[1])**2)**1.5) / np.absolute(2*right_fit[0])

        curvature = (left_curverad + right_curverad) / 2

        rospy.loginfo('Left curvature: %s, Right curvature: %s, Average curvature: %s' % (left_curverad, right_curverad, curvature))

        return curvature

    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height
        x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

        return heading_image

    def length_of_line_segment(self, line):
        x1, y1, x2, y2 = line
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def show_image(self, title, frame):
        if frame is None or frame.size == 0:
            rospy.logerr("Error showing image '{}': The image is empty or not initialized.".format(title))
            return
        cv2.imshow(title, frame)
        cv2.waitKey(1)

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height
        y2 = int(y1 * 1 / 2)

        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]

def main():
    rospy.init_node('lane_detect_node', anonymous=True)
    
    lane_detector = JdOpencvLaneDetect()
    bridge = CvBridge()

    def image_callback(msg):
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        lane_lines, lane_lines_image = lane_detector.get_lane(frame)
        steering_angle, heading_image = lane_detector.get_steering_angle(frame, lane_lines)
        curvature = lane_detector.get_curvature(frame, lane_lines)
        if curvature is not None:
            rospy.loginfo('Curvature: %s' % curvature)

        # Publish steering angle
        steering_angle_msg = Float32()
        steering_angle_msg.data = (steering_angle - 90) / 45  # Normalize the steering angle to the range [-1, 1]
        steering_pub.publish(steering_angle_msg)

        # Combine original frame with lane lines
        combined_image = cv2.addWeighted(frame, 0.8, lane_lines_image, 1, 0)

        # Show the image with lane lines
        lane_detector.show_image("Lane Detection", combined_image)

        # Publish the processed image
        processed_image_msg = bridge.cv2_to_imgmsg(combined_image, "bgr8")
        processed_image_pub.publish(processed_image_msg)

    # Subscribe to the camera image topic
    image_sub = rospy.Subscriber('/camera/color/image_raw', Image, image_callback)

    # Publisher for steering angle
    steering_pub = rospy.Publisher('/jetson_car/steering', Float32, queue_size=10)

    # Publisher for processed image
    processed_image_pub = rospy.Publisher('/lane_detection/image_raw', Image, queue_size=10)
    
    rospy.spin()

if __name__ == '__main__':
    main()
