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
import json
import numpy as np       
import supervision as sv
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO

names = ['car', 'crosswalk', 'highway_entry', 'highway_exit', 'no_entry', 'onewayroad', 'parking', 'pedestrian', 'priority', 'roadblock', 'roundabout', 'stop', 'trafficlight']

class CameraHandler():
    def __init__(self):
        """
        Creates a bridge for converting the image from Gazebo image intro OpenCv image
        """
        self.bridge = CvBridge()
        self.cv_image = np.zeros((640, 480))
        self.prev_sign = 0
        self.box_annotator = sv.BoxAnnotator(
            thickness=2,
            text_thickness=2,
            text_scale=1
        )
        rospy.init_node('SignDetection', anonymous=True)
        self.image_sub = rospy.Subscriber("/automobile/image_raw", Image, self.callback)
        self.sign_pub = rospy.Publisher("/control/sign", String, queue_size=20)
        self.rate = rospy.Rate(20)
        self.model = YOLO("./best.pt", task='detect')
        rospy.spin()

    def callback(self, data):
        """
        :param data: sensor_msg array containing the image in the Gazsbo format
        :return: nothing but sets [cv_image] to the usefull image that can be use in opencv (numpy array)
        """
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        results = self.model(self.cv_image, device="cpu", conf=0.6)

        detections = sv.Detections.from_yolov8(results[0])
        labels = [
            f"{self.model.model.names[class_id]} {confidence:0.2f}"
            for _, _, confidence, class_id, _
            in detections
        ]
        frame = self.box_annotator.annotate(
            scene=self.cv_image, 
            detections=detections, 
            labels=labels
        ) 
        cv2.imshow("cv_image", self.cv_image)
        cv2.waitKey(1)
        box_id = 0
        box_value = 20
        i = 0
        if len(results[0].boxes) == 0:
            if not self.prev_sign == "none":
                self.prev_sign = "none"
                self.sign_pub.publish("none")
            return
        for box in results[0].boxes:
            if box.xyxy[0][3] > 250:
                if results[0].boxes.cls[i] < box_value:
                    box_value = results[0].boxes.cls[i]
            elif results[0].boxes.cls[i] == 11:
                if box.xyxy[0][3] > 180:
                    box_value = 11
            elif results[0].boxes.cls[i] == 6:
                if box.xyxy[0][3] > 180 and box.xyxy[0][0] > 150:
                    box_value = 6
            elif results[0].boxes.cls[i] == 2:
                if box.xyxy[0][3] > 180:
                    box_value = 2
            elif results[0].boxes.cls[i] == 3:
                if box.xyxy[0][3] > 100:
                    box_value = 3
            i += 1
        if box_value == 0:
            if box.xyxy[0][0] > 550 or box.xyxy[0][2] < 100:
                box_value = 20
        
        if box_value == 7:
            if box.xyxy[0][3] < 300:
                box_value = 20
        if box_value == 20: 
            if not self.prev_sign == 'none':
                self.prev_sign = "none"
                self.sign_pub.publish("none")
            return
        
        print(box_value)
        detected = names[int(box_value)]
        if self.prev_sign == detected:
            return
        self.sign_pub.publish(detected)
        self.prev_sign = detected

            
if __name__ == '__main__':
    try:
        nod = CameraHandler()
    except rospy.ROSInterruptException:
        pass