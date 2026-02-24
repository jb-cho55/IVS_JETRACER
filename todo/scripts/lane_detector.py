#!/usr/bin/env python3

import cv2
import numpy as np
from warper import Warper
from slidewindow import SlideWindow


class LaneDetector:
    """
    Input: BGR image (numpy, HxWx3)
    Output:
      - x_location: estimated lane-center x at sample_y (None if fail)
      - current_line: 'LEFT'/'RIGHT'/'MID'
      - debug: debug image dict
    """

    def __init__(
        self,
        lane_colors=("black",),
        hsv_low=(50, 50, 50),
        hsv_high=(255, 255, 255),
        use_base_hsv=False,
        black_low=(0, 0, 0),
        black_high=(180, 255, 70),
        blur_ksize=5,
        use_morph=True,
        use_edge=True,
        canny_low=50,
        canny_high=150,
        sample_y=340,
    ):
        self.lane_colors = self._normalize_lane_colors(lane_colors)
        self.hsv_low = np.array(hsv_low, dtype=np.uint8)
        self.hsv_high = np.array(hsv_high, dtype=np.uint8)
        self.use_base_hsv = bool(use_base_hsv)
        self.color_ranges = {
            "white": (
                np.array([0, 0, 200], dtype=np.uint8),
                np.array([180, 80, 255], dtype=np.uint8),
            ),
            "yellow": (
                np.array([15, 70, 70], dtype=np.uint8),
                np.array([40, 255, 255], dtype=np.uint8),
            ),
            "black": (
                np.array(black_low, dtype=np.uint8),
                np.array(black_high, dtype=np.uint8),
            ),
        }
        self.blur_ksize = int(blur_ksize)
        self.use_morph = bool(use_morph)
        self.use_edge = bool(use_edge)
        self.canny_low = int(canny_low)
        self.canny_high = int(canny_high)
        self.sample_y = int(sample_y)

        self.warper = None
        self.slidewindow = SlideWindow(sample_y=self.sample_y)

    def _normalize_lane_colors(self, lane_colors):
        supported = {"white", "yellow", "black"}

        if isinstance(lane_colors, str):
            colors = [c.strip().lower() for c in lane_colors.split(",") if c.strip()]
        else:
            colors = [str(c).strip().lower() for c in lane_colors if str(c).strip()]

        if not colors:
            colors = ["black"]

        invalid = [c for c in colors if c not in supported]
        if invalid:
            raise ValueError(f"Unsupported lane_colors: {invalid}. Supported: {sorted(supported)}")

        # Remove duplicates but keep order.
        return tuple(dict.fromkeys(colors))

    def _ensure_warper(self, w: int, h: int):
        if self.warper is None or self.warper.w != w or self.warper.h != h:
            self.warper = Warper(w=w, h=h)

    def preprocess(self, bgr: np.ndarray) -> np.ndarray:
        """Build robust binary from color + edge."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

        if self.use_base_hsv:
            base_mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)
            mask = cv2.bitwise_or(mask, base_mask)

        for color in self.lane_colors:
            low, high = self.color_ranges[color]
            color_mask = cv2.inRange(hsv, low, high)
            mask = cv2.bitwise_or(mask, color_mask)

        if self.use_edge:
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
            edge_mask = cv2.Canny(gray, self.canny_low, self.canny_high)
            mask = cv2.bitwise_or(mask, edge_mask)

        if self.use_morph:
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

        return mask

    def process(self, bgr: np.ndarray):
        """Full lane-detection pipeline."""
        if bgr is None or bgr.size == 0:
            return None, "MID", {}

        h, w = bgr.shape[:2]
        self._ensure_warper(w, h)

        mask = self.preprocess(bgr)

        k = self.blur_ksize
        if k % 2 == 0:
            k += 1
        blur = cv2.GaussianBlur(mask, (k, k), 0)

        warped = self.warper.warp(blur)
        slide_img, x_location, current_line = self.slidewindow.slidewindow(warped)

        debug = {
            "mask": mask,
            "blur": blur,
            "warped": warped,
            "slide": slide_img,
        }
        return x_location, current_line, debug

    def compute_steering(self, x_location, target_x, k_steer=0.003, steer_limit=0.34):
        if x_location is None:
            return 0.0
        error = float(target_x - x_location)
        steer = error * float(k_steer)
        if steer > steer_limit:
            steer = steer_limit
        elif steer < -steer_limit:
            steer = -steer_limit
        return steer
