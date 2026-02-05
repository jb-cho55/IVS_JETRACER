#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


# =========================
# cv_bridge-free converters
# =========================
def rosimg_to_bgr(msg: Image):
    """
    Convert sensor_msgs/Image to OpenCV BGR image without cv_bridge.
    Supports common encodings from usb_cam: bgr8/rgb8/mono8/yuyv/yuv422(yuy2).
    """
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()

    if enc == "bgr8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return img.copy()

    if enc == "rgb8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if enc == "mono8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # usb_cam frequently publishes YUYV/YUV422
    if "yuy" in enc or "yuv422" in enc:
        # YUYV packed: 2 bytes per pixel -> shape (h, w, 2)
        yuyv = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 2)
        return cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUY2)

    raise ValueError(f"Unsupported encoding: {msg.encoding}")


def bgr_to_imgmsg(bgr, header=None):
    """
    Convert OpenCV BGR image to sensor_msgs/Image without cv_bridge.
    """
    out = Image()
    if header is not None:
        out.header = header
    out.height, out.width = bgr.shape[:2]
    out.encoding = "bgr8"
    out.is_bigendian = 0
    out.step = out.width * 3
    out.data = bgr.tobytes()
    return out


class JdOpencvLaneDetect(object):
    def __init__(self):
        self.curr_steering_angle = 90  # 90deg = straight

    def get_lane(self, frame):
        lane_lines, lane_lines_image = self.detect_lane(frame)
        return lane_lines, lane_lines_image

    def get_steering_angle(self, img_lane, lane_lines):
        if len(lane_lines) == 0:
            return 0, None

        new_steering_angle = self.compute_steering_angle(img_lane, lane_lines)
        self.curr_steering_angle = self.stabilize_steering_angle(
            self.curr_steering_angle, new_steering_angle, len(lane_lines)
        )

        curr_heading_image = self.display_heading_line(img_lane, self.curr_steering_angle)
        return self.curr_steering_angle, curr_heading_image

    def get_curvature(self, frame, lane_lines):
        if len(lane_lines) == 0:
            return None
        return self.compute_curvature(frame, lane_lines)

    # ------------------------------
    # Lane detection pipeline
    # ------------------------------
    def detect_lane(self, frame):
        edges = self.detect_edges(frame)
        if edges is None or edges.size == 0:
            rospy.logwarn("No edges detected.")
            return [], frame

        cropped_edges = self.region_of_interest(edges)
        if cropped_edges is None or cropped_edges.size == 0:
            rospy.logwarn("No cropped edges.")
            return [], frame

        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(frame, line_segments)
        lane_lines_image = self.display_lines(frame, lane_lines)
        return lane_lines, lane_lines_image

    def detect_edges(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # red mask (tune if needed)
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
            (0, int(height * 0.5)),
            (width, int(height * 0.5)),
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
        line_segments = cv2.HoughLinesP(
            cropped_edges, rho, angle, min_threshold,
            np.array([]), minLineLength=15, maxLineGap=4
        )
        return line_segments

    def average_slope_intercept(self, frame, line_segments):
        lane_lines = []
        if line_segments is None or len(line_segments) == 0:
            rospy.loginfo("No line segments detected")
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1 / 3
        left_region_boundary = width * (1 - boundary)
        right_region_boundary = width * boundary

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    continue

                slope, intercept = np.polyfit((x1, x2), (y1, y2), 1)

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

        return lane_lines

    # ------------------------------
    # Steering / curvature
    # ------------------------------
    def compute_steering_angle(self, frame, lane_lines):
        """
        Return steering angle in degrees:
        90 = straight, <90 = left, >90 = right
        """
        height, width, _ = frame.shape

        if len(lane_lines) == 1:
            x1, y1, x2, y2 = lane_lines[0][0]
            x_offset = x2 - x1
        else:
            left_x2 = lane_lines[0][0][2]
            right_x2 = lane_lines[1][0][2]
            mid = width / 2.0
            x_offset = (left_x2 + right_x2) / 2.0 - mid

        y_offset = height / 2.0
        angle_to_mid_radian = math.atan2(x_offset, y_offset)
        angle_to_mid_deg = angle_to_mid_radian * 180.0 / math.pi

        steering_angle = int(angle_to_mid_deg + 90)

        # clamp to avoid extreme angles
        steering_angle = max(45, min(135, steering_angle))
        return steering_angle

    def stabilize_steering_angle(
        self, curr_steering_angle, new_steering_angle, num_of_lane_lines,
        max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1
    ):
        max_angle_deviation = max_angle_deviation_two_lines if num_of_lane_lines == 2 else max_angle_deviation_one_lane
        angle_deviation = new_steering_angle - curr_steering_angle

        if abs(angle_deviation) > max_angle_deviation:
            stabilized = int(curr_steering_angle + max_angle_deviation * angle_deviation / abs(angle_deviation))
        else:
            stabilized = new_steering_angle

        rospy.loginfo("Proposed angle: %s, stabilized angle: %s", new_steering_angle, stabilized)
        return stabilized

    def compute_curvature(self, frame, lane_lines):
        if len(lane_lines) < 2:
            return None

        left_fit = np.polyfit(
            (lane_lines[0][0][1], lane_lines[0][0][3]),
            (lane_lines[0][0][0], lane_lines[0][0][2]), 2
        )
        right_fit = np.polyfit(
            (lane_lines[1][0][1], lane_lines[1][0][3]),
            (lane_lines[1][0][0], lane_lines[1][0][2]), 2
        )

        y_eval = frame.shape[0]
        left_curverad = ((1 + (2 * left_fit[0] * y_eval + left_fit[1]) ** 2) ** 1.5) / np.absolute(2 * left_fit[0])
        right_curverad = ((1 + (2 * right_fit[0] * y_eval + right_fit[1]) ** 2) ** 1.5) / np.absolute(2 * right_fit[0])

        return (left_curverad + right_curverad) / 2.0

    # ------------------------------
    # Visualization helpers
    # ------------------------------
    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        return cv2.addWeighted(frame, 0.8, line_image, 1, 1)

    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5):
        heading_image = np.zeros_like(frame)
        height, width, _ = frame.shape

        steering_angle_radian = steering_angle / 180.0 * math.pi
        x1 = int(width / 2)
        y1 = height

        tan_val = math.tan(steering_angle_radian)
        if abs(tan_val) < 1e-6:
            x2 = x1
        else:
            x2 = int(x1 - (height / 2) / tan_val)

        y2 = int(height / 2)

        cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
        return cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    def show_image(self, title, frame):
        if frame is None or frame.size == 0:
            return
        cv2.imshow(title, frame)
        cv2.waitKey(1)

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line

        y1 = height
        y2 = int(height * 0.5)

        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]


def main():
    rospy.init_node("lane_detect_node", anonymous=True)

    # parameters (can be overridden in launch)
    input_topic = rospy.get_param("~input_topic", "/usb_cam/image_raw")
    output_image_topic = rospy.get_param("~output_image_topic", "/lane_detection/image_raw")
    output_steer_topic = rospy.get_param("~output_steer_topic", "/Steering")  # actuator subscribes this in your setup
    show_window = rospy.get_param("~show_window", False)

    lane_detector = JdOpencvLaneDetect()

    steering_pub = rospy.Publisher(output_steer_topic, Float32, queue_size=10)
    processed_image_pub = rospy.Publisher(output_image_topic, Image, queue_size=1)

    if show_window:
        cv2.namedWindow("Lane Detection", cv2.WINDOW_NORMAL)

    def image_callback(msg: Image):
        try:
            frame = rosimg_to_bgr(msg)
        except Exception as e:
            rospy.logwarn("Image convert failed (encoding=%s): %s", msg.encoding, e)
            return

        lane_lines, lane_lines_image = lane_detector.get_lane(frame)
        steering_angle, heading_image = lane_detector.get_steering_angle(frame, lane_lines)

        # publish steering (-1~1)
        steering_angle_msg = Float32()
        steering_angle_msg.data = (steering_angle - 90) / 45.0
        steering_angle_msg.data = max(-1.0, min(1.0, steering_angle_msg.data))
        steering_pub.publish(steering_angle_msg)

        # visualization image
        vis = lane_lines_image
        if heading_image is not None:
            vis = cv2.addWeighted(vis, 0.8, heading_image, 1, 0)

        if show_window:
            lane_detector.show_image("Lane Detection", vis)

        # publish processed image for rqt_image_view/image_view
        processed_image_pub.publish(bgr_to_imgmsg(vis, header=msg.header))

    rospy.Subscriber(input_topic, Image, image_callback, queue_size=1, buff_size=2**24)

    rospy.loginfo("Lane node started. input=%s, out_image=%s, out_steer=%s",
                  input_topic, output_image_topic, output_steer_topic)
    rospy.spin()


if __name__ == "__main__":
    main()
