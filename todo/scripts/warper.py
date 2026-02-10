# warper.py
#!/usr/bin/env python3

import cv2
import numpy as np


class Warper:
    def __init__(self, w: int, h: int, src=None, dst=None):
        self.w = int(w)
        self.h = int(h)

        # 기본값: 기존 코드 비율 유지 (프레임 크기 따라 자동 스케일)
        if src is None:
            src = np.float32([
                [self.w * 1.6, self.h * 1.3],
                [self.w * (-0.1), self.h * 1.3],
                [0, self.h * 0.62],
                [self.w, self.h * 0.62],
            ])
        if dst is None:
            dst = np.float32([
                [self.w * 0.65, self.h * 0.98],
                [self.w * 0.35, self.h * 0.98],
                [self.w * (-0.3), 0],
                [self.w * 1.3, 0],
            ])

        self.M = cv2.getPerspectiveTransform(src, dst)
        self.Minv = cv2.getPerspectiveTransform(dst, src)

    def warp(self, img):
        return cv2.warpPerspective(img, self.M, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)

    def unwarp(self, img):
        return cv2.warpPerspective(img, self.Minv, (img.shape[1], img.shape[0]), flags=cv2.INTER_LINEAR)