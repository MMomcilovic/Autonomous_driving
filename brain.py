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

import json

from RcBrainThread import RcBrainThread
from std_msgs.msg import String, Byte
from utils.msg import localisation
from utils.msg import IMU

import rospy

class RemoteControlTransmitterProcess():
    # ===================================== INIT==========================================
    def __init__(self):
        """Run on the PC. It forwards the commans from the user via KeboardListenerThread to the RcBrainThread. 
        The RcBrainThread converts them into actual commands and sends them to the remote via a socket connection.
        320
        """
        self.data = {}
        self.rcBrain   =  RcBrainThread()   
        self.changeAngle = True
        self.shouldCheckLanes = True
        self.trafficLightValue = 2
        self.dontCheck = False
        self.posA = 0
        self.pobB = 0
        self.checkLight = False
        self.stopped = False
        self.stoppedOnSign = False
        self.parkingInProgress = False
        self.parked = False
        self.carSpeed = 20
        self.sleep = False
        self.firstCar = True
        self.inRoundabout = False
        self.onHighway = False
        rospy.init_node('EXAMPLEnode', anonymous=False)     
        self.publisher = rospy.Publisher('/automobile/command', String, queue_size=0)
        rospy.Subscriber("/automobile/localisation", localisation, self._loc)
        rospy.Subscriber("/control/coord", String, self._lane_detection)
        self.subSing = rospy.Subscriber("/control/sign", String, self._check_sign)
        rospy.Subscriber("/automobile/IMU", IMU, self._IMU)
        rospy.Subscriber("/automobile/feedback", String, self._feedback)
        self.subTraffic = rospy.Subscriber("/automobile/trafficlight/start", Byte, self._traffic)
        rospy.spin()

    def _loc(self, StringData):
        posA = StringData.posA
        posB = StringData.posB
        self.posA = posA
        self.posB = posB
        if posA > 0.7 and posA < 0.95 and posB > 14.7 and posB < 14.8:
            self.shouldCheckLanes = False
            self._keepForward()
            self.dontCheck = True
        elif posA > 0.7 and posA < 0.95 and posB > 13.1 and posB < 13.3:
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 0.7 and posA < 0.95 and posB > 11.15 and posB < 11.43:
            self.shouldCheckLanes = False
            self._rightTurn()
            self.dontCheck = True
        elif posA > 1.65 and posA < 1.88 and posB > 10.7 and posB < 11:
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 2.0 and posA < 2.3 and posB > 10.7 and posB < 11:
            self.shouldCheckLanes = False
            self._closeLeftTurn()
            self.dontCheck = True
        elif posA > 2.52 and posA < 2.72 and posB > 10 and posB < 10.35:
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 2.85 and posA < 3.15 and posB > 7.5 and posB < 7.65:
            self.shouldCheckLanes = False
            self._leftTurn()
            self.dontCheck = True
        elif posA > 1.7 and posA < 1.85 and posB > 6.5 and posB < 6.7:
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 1.22 and posA < 1.45 and posB > 6.50 and posB < 6.7: 
            self.shouldCheckLanes = False
            self._rightTurn()
            self.dontCheck = True
        elif posA > 0.7 and posA < 0.9 and posB > 5.65 and posB < 5.85:
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 4 and posA < 4.2 and posB > 1.98 and posB < 2.2:
            if (self.parked):
                return
            self.parked = True
            self.parkingInProgress = True
            self.shouldCheckLanes = False
            self._parkBackwards()
            self.dontCheck = True
        
        elif posA > 3.1 and posA < 3.4 and posB > 2.6 and posB < 2.75:
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()  
            self.data['action']        =  '2'
            self.data['steerAngle']    =  0.0
            self.controlCar()
            rospy.sleep(0.5)
            self._rightTurnWithoutStop()
            rospy.sleep(2)
            self.parkingInProgress = False
            self.shouldCheckLanes = True
            self.dontCheck = False

        elif posA > 0.45 and posA < 0.6 and posB > 14.6 and posB < 14.75:
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()  
            self.data['action']        =  '2'
            self.data['steerAngle']    =  0.0
            self.controlCar()
            self.dontCheck = True
            self.shouldCheckLanes = False

        elif posA > 10.3 and posA < 10.8 and posB > 3.4 and posB < 3.6:
            if not self.inRoundabout:
                self.shouldCheckLanes = True
                self.dontCheck = True
                self.inRoundabout = True
                self.data['action']        =  '1'
                self.data['speed']         =  float(15.0/100.0)
                self.controlCar()  
                self.data['action']        =  '2'
                self.data['steerAngle']    =  float(20.0)
                self.controlCar()
                rospy.sleep(8)
                self.data['action']        =  '2'
                self.data['steerAngle']    =  float(-21.0)
                self.controlCar()
        elif posA > 9.37 and posA < 9.55 and posB > 4.32 and posB < 4.45:
            self.inRoundabout = False
            self.shouldCheckLanes = True
            self.dontCheck = False
        elif posA > 6.25 and posA < 6.4 and posB >13.3 and posB < 13.45:
            self.shouldCheckLanes = False
            self._keepForward()
            self.dontCheck = True
        elif posA > 1.25 and posA < 1.46 and posB > 13.2 and posB < 13.45:
            self.shouldCheckLanes = False
            self._leftTurn()
            self.dontCheck = True
        elif posA > 0.4 and posA < 0.6 and posB > 14 and posB < 14.2:
            self.shouldCheckLanes = True
            self.dontCheck = False



    
    def _lane_detection(self, Stringdata):
        if not self.shouldCheckLanes or self.dontCheck:
            return
        self.stopped = False
        self.stoppedOnSign = False

        turnAngle = (int(Stringdata.data) - 320)
        carSpeed = self.carSpeed
        if abs(turnAngle) > 150:
            carSpeed = 15
        
        turnAngle = turnAngle/11
        
        if turnAngle > 21:
            turnAngle = 21

        self.data = {}

        self.data['action']        =  '1'
        self.data['speed']         =  float(carSpeed/100.00)
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle']    =  float(turnAngle)

        self.controlCar()

    def _check_sign(self, Stringdata):
        print(Stringdata)
        if (self.dontCheck):
            return
        if Stringdata.data == "trafficlight":
            if self.posA > 2 and self.posA < 2.15 and self.posB > 10.7 and self.posB < 11:
                self.subTraffic.unregister()
                self.subTraffic = rospy.Subscriber("/automobile/trafficlight/slave", Byte, self._traffic)
                self.shouldCheckLanes = False
            self.checkLight = True
        elif Stringdata.data == "pedestrian":
            self.shouldCheckLanes = False
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()
        elif Stringdata.data == "crosswalk":
            self.data['action']        =  '1'
            self.data['speed']         =  10.0
            self.controlCar()
        elif Stringdata.data == 'roadblock':
            self.shouldCheckLanes = False
            self.data['speed']         =  float(10.0/100.0)
            self.data['action']        =  '1'
            self.moveCar()
            self.data['action']        =  '2'
            self.data['steerAngle'] = float(20.0)
            self.controlCar()
        elif Stringdata.data == 'stop':
            if self.stoppedOnSign:
                return
            self.shouldCheckLanes = False
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()
            self.stoppedOnSign = True
            rospy.sleep(3)
            self.shouldCheckLanes = True
        elif Stringdata.data == 'parking':
            print('parking')
        elif Stringdata.data == 'car' and self.posA > 4:
            print('aa')
            if (self.posB > 2 and self.posB < 2.2) or (self.posA > 9.2 and self.posA < 9.5):
                self.goToLeftLane()
            else:
                self.goToRightLane()
        elif Stringdata.data == 'highway_entry':
            self.onHighway = True
            self.carSpeed = 30
        elif Stringdata.data == 'highway_exit':
            self.onHighway = False
            self.carSpeed = 20
        else:
            self.shouldCheckLanes = True
            self.checkLight = False

    def _IMU(self, data):
        pass
        # print("IMU", data)

    def _feedback(self, data):
        pass
        # print(data)
     
    def _traffic(self, data):
        if (self.checkLight):
            self.trafficLightValue = data.data
        pass
    
    def _closeLeftTurn(self):
        self.stopCar()
        self.stopped = True
        self.data['speed']         =  float(15.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(-21.0)
        self.controlCar()

    def _leftTurn(self):
        self.stopCar()
        if not self.stopped:
            rospy.sleep(0.5)
        self.stopped = True
        self.data['speed']         =  float(15.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(-13.0)
        self.controlCar()

    def _rightTurn(self):
        self.stopCar()
        if not self.stopped:
            self.stopped = True
            rospy.sleep(0.5)
        self.data['speed']         =  float(15.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(20.0)
        self.controlCar()

    def _rightTurnWithoutStop(self):
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(20.0)
        self.controlCar()
        rospy.sleep(0.1)
        self.data['action']        =  '1'
        self.data['speed']         =  float(15.0/100.0)
        self.moveCar()


    def _parkBackwards(self):
        self.stopped = True
        print('tu sam')
        self.data['speed']         =  float(0.0/100.0)
        self.data['action']        =  '1'
        self.controlCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(21.0)
        self.controlCar()
        rospy.sleep(0.1)
        self.data['speed']         =  float(-15.0/100.0)
        self.data['action']        =  '1'
        self.controlCar()

    def _parkBackwards2(self):
        self.stopped = True
        print('tu sam2')
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(21.0)
        self.controlCar()

    def _keepForward(self):
        self.data['speed']         =  float(15.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(0.0)
        self.controlCar()

    def goToLeftLane(self):
        self.shouldCheckLanes = False
        self.firstCar = False
        if (self.onHighway):
            self.data['speed']         =  float(15.0/100.0)
        else:
            self.data['speed']         =  float(8.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(-20.0)
        self.controlCar()

    def goToRightLane(self):
        self.shouldCheckLanes = False
        self.firstCar = True
        if (self.onHighway):
            self.data['speed']         =  float(15.0/100.0)
        else:
            self.data['speed']         =  float(8.0/100.0)
        self.data['action']        =  '1'
        self.moveCar()
        self.data['action']        =  '2'
        self.data['steerAngle'] = float(20.0)
        self.controlCar()

    def stopCar(self):
        if not self.stopped:
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()

    def moveCar(self):
        if self.checkLight and not self.trafficLightValue == 2:
            self.data['action']        =  '1'
            self.data['speed']         =  0.0
            self.controlCar()
            return
        self.controlCar()

    def controlCar(self):
        command = json.dumps(self.data)
        self.publisher.publish(command)  
        self.data = {}

if __name__ == '__main__':
    try:
        nod = RemoteControlTransmitterProcess()
    except rospy.ROSInterruptException:
        pass
