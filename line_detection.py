#!/usr/bin/env python3

# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE


import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

def pipeline(img, s_thresh=(100, 255), sx_thresh=(15, 255)):
    img = np.copy(img)
    # Convert to HLS color space and separate the V channel
    hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS).astype(np.float32)
    l_channel = hls[:,:,1]
    s_channel = hls[:,:,2]
    h_channel = hls[:,:,0]

    # Sobel x
    sobelx = cv2.Sobel(l_channel, cv2.CV_64F, 1, 1) # Take the derivative in x
    abs_sobelx = np.absolute(sobelx) # Absolute x derivative to accentuate lines away from horizontal
    scaled_sobel = np.uint8(255*abs_sobelx/np.max(abs_sobelx))
    
    # Threshold x gradient
    sxbinary = np.zeros_like(scaled_sobel)
    sxbinary[(scaled_sobel >= sx_thresh[0]) & (scaled_sobel <= sx_thresh[1])] = 1
    
    # Threshold color channel
    s_binary = np.zeros_like(s_channel)
    s_binary[(s_channel >= s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    
    color_binary = np.dstack((np.zeros_like(sxbinary), sxbinary, s_binary)) * 255

    return color_binary

def perspective_warp(img, 
                     dst_size=(640,220),
                     src=np.float32([(0,1),(1,1),(0,0),(1,0)]),
                     dst=np.float32([(0.3,1), (0.7, 1), (0,0), (1,0)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def inv_perspective_warp(img, 
                     dst_size=(640,220),
                     dst=np.float32([(0,1),(0.95,1),(0,0),(1,0)]),
                     src=np.float32([(0.45,1), (0.6, 1), (0,0), (1,0)])):
    img_size = np.float32([(img.shape[1],img.shape[0])])
    src = src* img_size
    # For destination points, I'm arbitrarily choosing some points to be
    # a nice fit for displaying our warped result 
    # again, not exact, but close enough for our purposes
    dst = dst * np.float32(dst_size)
    # Given src and dst points, calculate the perspective transform matrix
    M = cv2.getPerspectiveTransform(src, dst)
    # Warp the image using OpenCV warpPerspective()
    warped = cv2.warpPerspective(img, M, dst_size)
    return warped

def get_hist(img):
    hist = np.sum(img[img.shape[0]//2:,:], axis=0)
    return hist


def sliding_window(img, nwindows=9, margin=50, minpix = 30, draw_windows=True):
    out_img = np.dstack((img, img, img))*255

    histogram = get_hist(img[100:220, 0:640])
    # find peaks of left and right halves
    midpoint = int(histogram.shape[0]/2)
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint + 20:]) + midpoint
    if rightx_base == 320:
        rightx_base = 640
    
    
    # Set height of windows
    window_height = np.int32(img.shape[0]/nwindows)
    # Identify the x and y positions of all nonzero pixels in the image
    nonzero = img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    # Current positions to be updated for each window
    leftx_current = leftx_base
    rightx_current = rightx_base
    
    
    # Create empty lists to receive left and right lane pixel indices
    left_lane_inds = []
    right_lane_inds = []

    # Step through the windows one by one
    for window in range(nwindows):
        # Identify window boundaries in x and y (and right and left)
        win_y_low = img.shape[0] - (window+1)*window_height
        win_y_high = img.shape[0] - window*window_height
        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        # Draw the windows on the visualization image
        if draw_windows == True:
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),(win_xleft_high,win_y_high),
            (100,255,255), 3) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),(win_xright_high,win_y_high),
            (100,255,255), 3) 
        # Identify the nonzero pixels in x and y within the window
        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
        (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
        # Append these indices to the lists
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        # If you found > minpix pixels, recenter next window on their mean position
        if len(good_left_inds) > minpix:
            leftx_current = np.int32(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:        
            rightx_current = np.int32(np.mean(nonzerox[good_right_inds]))
        

    # Concatenate the arrays of indices
    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)

    # Extract left and right line pixel positions
    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds] 
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds] 

    # Fit a second order polynomial to each
    if len(lefty) != 0 and len(leftx) != 0:
        left_fit = np.polyfit(lefty, leftx, 2)
    else:
        left_fit = [0,0,0]
    if len(righty) != 0 and len(rightx) != 0:
        right_fit = np.polyfit(righty, rightx, 2)
    else:
        right_fit = [0,0,0]
    
    # Generate x and y values for plotting
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0] )
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    ltx = np.trunc(left_fitx)
    rtx = np.trunc(right_fitx)

    out_img[nonzeroy[left_lane_inds], nonzerox[left_lane_inds]] = [255, 0, 50]
    out_img[nonzeroy[right_lane_inds], nonzerox[right_lane_inds]] = [0, 50, 255]
    
    return out_img, (ltx, rtx), (left_fit, right_fit), ploty

def draw_lanes(img, left_fit, right_fit):
    ploty = np.linspace(0, img.shape[0]-1, img.shape[0])
    color_img = np.zeros_like(img)
    
    left = np.array([np.transpose(np.vstack([left_fit, ploty]))])
    right = np.array([np.flipud(np.transpose(np.vstack([right_fit, ploty])))])
    points = np.hstack((left, right))
    
    cv2.fillPoly(color_img, np.int_(points), (0,200,255))
    inv_perspective = inv_perspective_warp(color_img)
    inv_perspective = cv2.addWeighted(img, 1, inv_perspective, 0.7, 0)
    return inv_perspective

class CameraHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        self.pre_x = 320
        rospy.init_node('LineDetection', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.coord_pub = rospy.Publisher("/control/coord", String, queue_size=10)
        self.rate = rospy.Rate(20)
        rospy.spin()

    def callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        img = self.cv_image

        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        pipe = pipeline(img)
        cv2.imshow('first', img)
        cv2.imshow('pipeline', pipe)
        perspective = perspective_warp(pipe[250:470, 0:640])
        cv2.imshow("perspektiva", perspective)

        gray = cv2.cvtColor(perspective, cv2.COLOR_BGR2GRAY)

        out_img, curves, lanes, ploty = sliding_window(gray)
        cv2.imshow('sliding', out_img)
        meanx = int(curves[0][100] + abs(curves[0][100] - curves[1][100]) / 2)
        if (curves[0][100] == 0 and curves[1][100] == 0):
            meanx = 320
        elif meanx < 0:
            meanx = 0
        elif meanx > 640:
            meanx = 640
        if abs(meanx - self.pre_x) < 20 and abs(meanx - self.pre_x) > 250:
            meanx = self.pre_x
        self.pre_x = meanx
        image = cv2.circle(self.cv_image, (meanx,350), radius=1, color=(0, 0, 255), thickness=4)
        cv2.imshow('tackica', image)
        self.coord_pub.publish(str(meanx))

        cv2.waitKey(1)
    
            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        pass