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


def parse_lane_colors(raw):
    if isinstance(raw, str):
        colors = [c.strip() for c in raw.split(",") if c.strip()]
    elif isinstance(raw, (list, tuple)):
        colors = [str(c).strip() for c in raw if str(c).strip()]
    else:
        colors = []

    if not colors:
        return ["black"]
    return colors


class LaneToSteeringRawNode:
    def __init__(self):
        lane_colors_param = rospy.get_param("~lane_colors", rospy.get_param("~lane_color", "black"))
        # 차선 인식기
        self.detector = LaneDetector(
            lane_colors=parse_lane_colors(lane_colors_param),
            hsv_low=tuple(rospy.get_param("~hsv_low", [50, 50, 50])),
            hsv_high=tuple(rospy.get_param("~hsv_high", [255, 255, 255])),
            use_base_hsv=bool(rospy.get_param("~use_base_hsv", False)),
            black_low=tuple(rospy.get_param("~black_low", [0, 0, 0])),
            black_high=tuple(rospy.get_param("~black_high", [180, 255, 70])),
            blur_ksize=int(rospy.get_param("~blur_ksize", 5)),
            use_edge=bool(rospy.get_param("~use_edge", True)),
            canny_low=int(rospy.get_param("~canny_low", 50)),
            canny_high=int(rospy.get_param("~canny_high", 150)),
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
        rospy.loginfo(
            f"[LaneToSteeringRawNode] lane_colors={self.detector.lane_colors}, "
            f"use_base_hsv={self.detector.use_base_hsv}"
        )

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
            display = bgr.copy()

            slide = debug.get("slide")
            if slide is not None and self.detector.warper is not None:
                # Keep only colored sliding-window drawings and project them back.
                colored = (slide[:, :, 0] != slide[:, :, 1]) | (slide[:, :, 1] != slide[:, :, 2])
                slide_overlay = np.zeros_like(slide)
                slide_overlay[colored] = slide[colored]

                overlay_unwarped = self.detector.warper.unwarp(slide_overlay)
                overlay_mask = np.any(overlay_unwarped > 0, axis=2)
                blended = cv2.addWeighted(display, 1.0, overlay_unwarped, 0.9, 0.0)
                display[overlay_mask] = blended[overlay_mask]

            text = f"steering_cmd: {steer_deg:+.1f} deg"
            cv2.putText(display, text, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(display, text, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)

            cv2.imshow("lane_follow_debug", display)
            cv2.waitKey(1)


def main():
    rospy.init_node("lane_to_steering_raw", anonymous=False)
    LaneToSteeringRawNode()
    rospy.spin()


if __name__ == "__main__":
    main()
