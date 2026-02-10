#!/usr/bin/env python3
import os, sys
sys.path.insert(0, os.path.dirname(__file__))
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from lane_detector import LaneDetector


def rosimg_to_bgr(msg: Image):
    """sensor_msgs/Image -> OpenCV BGR (cv_bridge 없이 변환)"""
    h, w = msg.height, msg.width
    enc = (msg.encoding or "").lower()

    if enc == "bgr8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3).copy()

    if enc == "rgb8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

    if enc == "mono8":
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # usb_cam에서 흔한 YUYV/YUV422
    if enc in ("yuyv", "yuv422", "yuy2"):
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 2)
        return cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2)

    raise ValueError(f"Unsupported encoding: {msg.encoding}")


class LaneToSteeringRawNode:
    def __init__(self):
        # 차선 인식기
        self.detector = LaneDetector(
            hsv_low=tuple(rospy.get_param("~hsv_low", [50, 50, 50])),
            hsv_high=tuple(rospy.get_param("~hsv_high", [255, 255, 255])),
            blur_ksize=int(rospy.get_param("~blur_ksize", 5)),
            sample_y=int(rospy.get_param("~sample_y", 340)),
        )

        self.show_debug = bool(rospy.get_param("~show_debug", True))

        # 조향 계산 파라미터 (기존 main.py 기반)
        self.target_x = int(rospy.get_param("~target_x", 280))
        self.k_steer = float(rospy.get_param("~k_steer", 0.003))

        # 내부 steer(rad) 제한(기존 -0.34~+0.34) → -90~+90도로 스케일
        self.steer_limit_rad = float(rospy.get_param("~steer_limit_rad", 0.34))
        self.max_deg = float(rospy.get_param("~max_deg", 90.0))

        # ✅ 구독/발행 토픽
        self.img_topic = rospy.get_param("~img_topic", "/usb_cam/image_raw")
        self.out_topic = rospy.get_param("~out_topic", "steering_raw")

        self.pub = rospy.Publisher(self.out_topic, Float32, queue_size=1)
        rospy.Subscriber(self.img_topic, Image, self.cb, queue_size=1)

        rospy.loginfo(f"[LaneToSteeringRawNode] subscribe: {self.img_topic}")
        rospy.loginfo(f"[LaneToSteeringRawNode] publish  : {self.out_topic} (deg -{self.max_deg}..+{self.max_deg})")

    def cb(self, msg: Image):
        try:
            bgr = rosimg_to_bgr(msg)
        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Image convert failed: {e}")
            return

        x_loc, which, debug = self.detector.process(bgr)

        # steer(rad) 계산 (LaneDetector.compute_steering는 steer_limit clamp 포함)
        steer_rad = self.detector.compute_steering(
            x_location=x_loc,
            target_x=self.target_x,
            k_steer=self.k_steer,
            steer_limit=self.steer_limit_rad,
        )

        # rad(-limit..+limit) -> deg(-90..+90) 스케일
        if self.steer_limit_rad > 1e-6:
            steer_deg = (steer_rad / self.steer_limit_rad) * self.max_deg
        else:
            steer_deg = 0.0

        # clamp 최종 보장
        if steer_deg > self.max_deg:
            steer_deg = self.max_deg
        elif steer_deg < -self.max_deg:
            steer_deg = -self.max_deg

        self.pub.publish(Float32(data=float(steer_deg)))

        if self.show_debug:
            cv2.imshow("mask", debug["mask"])
            cv2.imshow("warped", debug["warped"])
            cv2.imshow("slide", debug["slide"])
            cv2.waitKey(1)


def main():
    rospy.init_node("lane_to_steering_raw", anonymous=False)
    LaneToSteeringRawNode()
    rospy.spin()


if __name__ == "__main__":
    main()
