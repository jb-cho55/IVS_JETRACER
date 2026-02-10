# slidewindow.py
#!/usr/bin/env python3

import cv2
import numpy as np


class SlideWindow:
    def __init__(self, sample_y: int = 340):
        self.current_line = "DEFAULT"
        self.sample_y = int(sample_y)

    def slidewindow(self, img: np.ndarray):
        """
        img: warped binary image (0/255)
        return: (out_img, x_location, current_line)
        """
        h, w = img.shape[:2]
        out_img = np.dstack((img, img, img))

        nonzero = img.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        if len(nonzerox) == 0:
            self.current_line = "MID"
            return out_img, None, self.current_line

        # 기존 코드의 ROI 값 유지 (픽셀 기준)
        win_h1, win_h2 = 400, 480
        win_l_w_l, win_l_w_r = 140, 240
        win_r_w_l, win_r_w_r = 310, 440

        # debug polyline
        pts_left = np.array([[win_l_w_l, win_h2], [win_l_w_l, win_h1], [win_l_w_r, win_h1], [win_l_w_r, win_h2]], np.int32)
        pts_right = np.array([[win_r_w_l, win_h2], [win_r_w_l, win_h1], [win_r_w_r, win_h1], [win_r_w_r, win_h2]], np.int32)
        cv2.polylines(out_img, [pts_left], False, (0, 255, 0), 1)
        cv2.polylines(out_img, [pts_right], False, (255, 0, 0), 1)
        cv2.line(out_img, (0, self.sample_y), (w, self.sample_y), (0, 120, 120), 1)

        good_left = ((nonzerox >= win_l_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_l_w_r)).nonzero()[0]
        good_right = ((nonzerox >= win_r_w_l) & (nonzeroy <= win_h2) & (nonzeroy > win_h1) & (nonzerox <= win_r_w_r)).nonzero()[0]

        margin = 20
        window_height = 7
        nwindows = 30

        x_location = None

        # 어느 쪽 라인이 더 강한지 선택
        if len(good_left) > len(good_right):
            self.current_line = "LEFT"
            line_flag = 1
            x_current = int(np.mean(nonzerox[good_left]))
            y_current = int(np.mean(nonzeroy[good_left]))
        elif len(good_right) > len(good_left):
            self.current_line = "RIGHT"
            line_flag = 2
            x_current = int(np.mean(nonzerox[good_right]))
            y_current = int(np.max(nonzeroy[good_right]))
        else:
            self.current_line = "MID"
            return out_img, None, self.current_line

        # sliding
        for window in range(nwindows):
            win_y_low = y_current - (window + 1) * window_height
            win_y_high = y_current - window * window_height

            win_x_low = x_current - margin
            win_x_high = x_current + margin

            if line_flag == 1:
                # LEFT: 현재창(녹색) + 오른쪽으로 차선폭만큼 이동한 창(파랑)
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low + int(w * 0.27), win_y_low), (win_x_high + int(w * 0.27), win_y_high), (255, 0, 0), 1)

                inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                        (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                if len(inds) > 0:
                    x_current = int(np.mean(nonzerox[inds]))

                if win_y_low <= self.sample_y <= win_y_high:
                    x_location = x_current + int(w * 0.135)

            else:
                # RIGHT: 현재창(파랑) + 왼쪽으로 차선폭만큼 이동한 창(녹색)
                cv2.rectangle(out_img, (win_x_low - int(w * 0.27), win_y_low), (win_x_high - int(w * 0.27), win_y_high), (0, 255, 0), 1)
                cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (255, 0, 0), 1)

                inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                        (nonzerox >= win_x_low) & (nonzerox < win_x_high)).nonzero()[0]
                if len(inds) > 0:
                    x_current = int(np.mean(nonzerox[inds]))

                if win_y_low <= self.sample_y <= win_y_high:
                    x_location = x_current - int(w * 0.135)

        return out_img, x_location, self.current_line
