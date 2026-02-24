#!/usr/bin/env python3

import cv2
import numpy as np


class SlideWindow:
    def __init__(self, sample_y: int = 340):
        self.current_line = "DEFAULT"
        self.sample_y = int(sample_y)
        self.prev_lane_width = None

    @staticmethod
    def _fit_x(poly, y):
        return poly[0] * (y ** 2) + poly[1] * y + poly[2]

    def slidewindow(self, img: np.ndarray):
        """
        img: warped binary image (0/255)
        return: (out_img, x_location, current_line)
        """
        h, w = img.shape[:2]
        out_img = np.dstack((img, img, img))
        cv2.line(out_img, (0, self.sample_y), (w, self.sample_y), (0, 120, 120), 1)

        binary = (img > 0).astype(np.uint8)
        nonzero = binary.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        if len(nonzerox) == 0:
            self.current_line = "MID"
            return out_img, None, self.current_line

        histogram = np.sum(binary[h // 2 :, :], axis=0)
        midpoint = w // 2
        leftx_base = int(np.argmax(histogram[:midpoint]))
        rightx_base = int(np.argmax(histogram[midpoint:]) + midpoint)

        nwindows = 12
        window_height = max(1, h // nwindows)
        margin = max(20, int(w * 0.08))
        minpix = max(20, int(w * 0.01))

        leftx_current = leftx_base
        rightx_current = rightx_base
        left_lane_inds = []
        right_lane_inds = []

        for window in range(nwindows):
            win_y_low = h - (window + 1) * window_height
            win_y_high = h - window * window_height

            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            cv2.rectangle(out_img, (win_xleft_low, win_y_low), (win_xleft_high, win_y_high), (0, 255, 0), 1)
            cv2.rectangle(out_img, (win_xright_low, win_y_low), (win_xright_high, win_y_high), (255, 0, 0), 1)

            good_left_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xleft_low)
                & (nonzerox < win_xleft_high)
            ).nonzero()[0]
            good_right_inds = (
                (nonzeroy >= win_y_low)
                & (nonzeroy < win_y_high)
                & (nonzerox >= win_xright_low)
                & (nonzerox < win_xright_high)
            ).nonzero()[0]

            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        left_lane_inds = np.concatenate(left_lane_inds) if left_lane_inds else np.array([], dtype=np.int64)
        right_lane_inds = np.concatenate(right_lane_inds) if right_lane_inds else np.array([], dtype=np.int64)

        left_fit = None
        right_fit = None

        if len(left_lane_inds) >= 100:
            leftx = nonzerox[left_lane_inds]
            lefty = nonzeroy[left_lane_inds]
            left_fit = np.polyfit(lefty, leftx, 2)

        if len(right_lane_inds) >= 100:
            rightx = nonzerox[right_lane_inds]
            righty = nonzeroy[right_lane_inds]
            right_fit = np.polyfit(righty, rightx, 2)

        x_location = None
        y_eval = int(np.clip(self.sample_y, 0, h - 1))

        if left_fit is not None and right_fit is not None:
            left_x = self._fit_x(left_fit, y_eval)
            right_x = self._fit_x(right_fit, y_eval)
            lane_width = right_x - left_x
            if 0.2 * w < lane_width < 0.8 * w:
                self.prev_lane_width = lane_width
            x_location = int((left_x + right_x) * 0.5)
            self.current_line = "MID"
        elif left_fit is not None:
            left_x = self._fit_x(left_fit, y_eval)
            lane_width = self.prev_lane_width if self.prev_lane_width is not None else 0.27 * w
            x_location = int(left_x + lane_width * 0.5)
            self.current_line = "LEFT"
        elif right_fit is not None:
            right_x = self._fit_x(right_fit, y_eval)
            lane_width = self.prev_lane_width if self.prev_lane_width is not None else 0.27 * w
            x_location = int(right_x - lane_width * 0.5)
            self.current_line = "RIGHT"
        else:
            self.current_line = "MID"
            return out_img, None, self.current_line

        x_location = int(np.clip(x_location, 0, w - 1))
        cv2.circle(out_img, (x_location, y_eval), 4, (0, 255, 255), -1)

        return out_img, x_location, self.current_line
