# lane_detector.py
#!/usr/bin/env python3

import cv2
import numpy as np
from warper import Warper
from slidewindow import SlideWindow


class LaneDetector:
    """
    입력: BGR 이미지 (numpy, HxWx3)
    출력:
      - x_location: 차선 중심(또는 목표 추종점) x 좌표 (없으면 None)
      - current_line: 'LEFT'/'RIGHT'/'MID'
      - debug: 디버그용 이미지 dict
    """
    def __init__(
        self,
        hsv_low=(50, 50, 50),
        hsv_high=(255, 255, 255),
        blur_ksize=5,
        use_morph=True,
        sample_y=340,          # x_location 뽑는 y 기준(기존 코드 338~344 근처)
    ):
        self.hsv_low = np.array(hsv_low, dtype=np.uint8)
        self.hsv_high = np.array(hsv_high, dtype=np.uint8)
        self.blur_ksize = int(blur_ksize)
        self.use_morph = bool(use_morph)
        self.sample_y = int(sample_y)

        self.warper = None           # 프레임 크기 확정 후 생성
        self.slidewindow = SlideWindow(sample_y=self.sample_y)

    def _ensure_warper(self, w: int, h: int):
        if self.warper is None or self.warper.w != w or self.warper.h != h:
            self.warper = Warper(w=w, h=h)

    def preprocess(self, bgr: np.ndarray) -> np.ndarray:
        """HSV 마스크(이진) 생성"""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_low, self.hsv_high)

        if self.use_morph:
            k = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

        return mask

    def process(self, bgr: np.ndarray):
        """전체 파이프라인"""
        if bgr is None or bgr.size == 0:
            return None, "MID", {}

        h, w = bgr.shape[:2]
        self._ensure_warper(w, h)

        mask = self.preprocess(bgr)

        # blur
        k = self.blur_ksize
        if k % 2 == 0:
            k += 1
        blur = cv2.GaussianBlur(mask, (k, k), 0)

        # warp
        warped = self.warper.warp(blur)

        # sliding window
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
