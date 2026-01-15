#!/usr/bin/env python

import cv2
import numpy as np

# Global variables
def perspective_transform(image):
    height, width = image.shape[:2]
    src = np.float32([
        [width // 2 - 500, height * 0.4],
        [width // 2 + 500, height * 0.4],
        [width, height],
        [0, height]
    ])
    dst = np.float32([
        [0, 0],
        [width, 0],
        [width, height],
        [0, height]
    ])
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(image, M, (width, height))
    return warped, M

def find_lane_pixels(binary_warped):
    histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] // 2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    nwindows = 9
    margin = 100
    minpix = 50
    window_height = int(binary_warped.shape[0] // nwindows)
    nonzero = binary_warped.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    leftx_current = leftx_base
    rightx_current = rightx_base

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = binary_warped.shape[0] - (window + 1) * window_height
        win_y_high = binary_warped.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]

    return leftx, lefty, rightx, righty, out_img, nonzerox, nonzeroy, left_lane_inds, right_lane_inds

def fit_polynomial(binary_warped):
    leftx, lefty, rightx, righty, out_img, nonzerox, nonzeroy, left_lane_inds, right_lane_inds = find_lane_pixels(binary_warped)
    left_fit = np.polyfit(lefty, leftx, 2) if len(leftx) > 0 and len(lefty) > 0 else [0, 0, 0]
    right_fit = np.polyfit(righty, rightx, 2) if len(rightx) > 0 and len(righty) > 0 else [0, 0, 0]

    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    try:
        left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
    except TypeError:
        left_fitx = 1 * ploty ** 2 + 1 * ploty
        right_fitx = 1 * ploty ** 2 + 1 * ploty

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 0]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 0, 255]

    return out_img, left_fit, right_fit

def measure_curvature_real(left_fit, right_fit, ploty):
    ym_per_pix = 30 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    y_eval = np.max(ploty)

    left_curverad = ((1 + (2 * left_fit[0] * y_eval * ym_per_pix + left_fit[1]) ** 2) ** 1.5) / np.abs(2 * left_fit[0])
    right_curverad = ((1 + (2 * right_fit[0] * y_eval * ym_per_pix + right_fit[1]) ** 2) ** 1.5) / np.abs(2 * right_fit[0])

    return left_curverad, right_curverad

def draw_lane_lines(original_image, binary_warped, left_fit, right_fit, Minv):
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
    color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (original_image.shape[1], original_image.shape[0]))
    result = cv2.addWeighted(original_image, 1, newwarp, 0.3, 0)

    return result

def mark_img(image, blue_threshold=200, green_threshold=200, red_threshold=200):
    bgr_threshold = [blue_threshold, green_threshold, red_threshold]
    thresholds = (image[:,:,0] < bgr_threshold[0]) | (image[:,:,1] < bgr_threshold[1]) | (image[:,:,2] < bgr_threshold[2])
    mark = np.copy(image)
    mark[thresholds] = [0, 0, 0]
    return mark

def process_video(file_path):
    cap = cv2.VideoCapture(file_path)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_no_yellow = mark_img(frame)
        gray = cv2.cvtColor(frame_no_yellow, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        warped, M = perspective_transform(edges)
        out_img, left_fit, right_fit = fit_polynomial(warped)

        ploty = np.linspace(0, warped.shape[0] - 1, warped.shape[0])
        left_curverad, right_curverad = measure_curvature_real(left_fit, right_fit, ploty)
        curvature = (left_curverad - right_curverad) / 2

        if (curvature < 200 and curvature > -200) or curvature >10000 or curvature<-10000:  # Higher values indicate straight road
            steering = 0
        else:
            steering = curvature * 0.0005

        Minv = np.linalg.inv(M)
        lane_image = draw_lane_lines(frame, warped, left_fit, right_fit, Minv)

        cv2.putText(lane_image, "Curve: {:.2f}m".format(curvature), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.putText(lane_image, "Steering: {:.2f}".format(steering), (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        cv2.imshow('Lane Detection', lane_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    video_path = 'example_1.mp4'
    process_video(video_path)
