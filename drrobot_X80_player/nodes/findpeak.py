#!/usr/bin/env python


import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3Stamped, Point
import pylab

# imu data
xdata = []
zdata = []

# peak data
ip = []
iv = []
ip_Location = []
iv_Location = []
PosX = 0
PosY = 0
valley = False
peak = False

# PDR
trajectory = []
steps = 0
walk_threshold = 0.4
max_walk_freq = 100  # 2sec(50Hz)
min_walk_freq = 3

# z-score smooth para
lag = 5
threshold = 5
influence = 0


class FINDPEAK():
    def __init__(self):
        rospy.init_node("realtime_findpeak")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Ready to found!")
        # Publisher to pose the target movement
        self.PDR = rospy.Publisher('/PDR_position', Point, queue_size=5)

        try:
            self.track = False
            self.track_subscriber = rospy.Subscriber(
                '/imu/rpy/complementary_filtered', Vector3Stamped, self.set_tracker, queue_size=1)
            rospy.wait_for_message(
                '/imu/rpy/complementary_filtered', Vector3Stamped)

        except rospy.ROSInterruptException:
            self.track = True
            rospy.loginfo("....")

    def set_tracker(self, position):
        posmsg = Point()
        global valley, peak, ip, iv, ip_Location, iv_Location, steps, trajectory, PosX, PosY,init_rot

        if self.track == False:
            # print("ok")
            z = position.vector.z  # yaw
            x = position.vector.x  # pitch
            xdata.append(x)
            zdata.append(z)

            if len(zdata) > 10 and len(zdata) < 12:
                init_rot = np.mean(zdata[1:10])
                print("init rot:", init_rot)

            if len(xdata) > lag+1:

                signal = thresholding_algo(xdata, lag, threshold, influence)
                i = len(signal)
                # print(i, signal[i-1])

                # print(signal['signals'])

                if signal[i-1] == -1:
                    if xdata[i-1] < xdata[i-2]:
                        iv = xdata[i-1]
                        iv_Location = i-1
                        valley = True
                        signal[i-1] = 0

                elif signal[i-1] == 1 and valley == True:
                    if xdata[i-1] > xdata[i-2]:
                        ip = xdata[i-1]
                        ip_Location = i-1
                        signal[i-1] = 0

                    elif xdata[i-1] < xdata[i-2]:
                        peak = True

                if (valley and peak) == True:
                    # print("ok")
                    #print(iv_Location, ip_Location, iv, ip)
                    # walking freq (between 0.1sec ~ 2sec(50Hz))
                    if (ip_Location - iv_Location) >= min_walk_freq and (ip_Location - iv_Location) < max_walk_freq and (ip-iv) > walk_threshold:
                        steps = steps + 1
                        print('steps =', steps)

                        [StepLength, PosX, PosY] = PosUpdata(
                            iv_Location, iv, ip_Location, ip, zdata, PosX, PosY, 0.023721, .383, 0)

                        PosRot_x, PosRot_y = RotMat(PosX, PosY, init_rot)
                        posmsg.x = PosRot_x
                        posmsg.y = PosRot_y
                        '''
                        posmsg.x = PosX
                        posmsg.y = PosY
                        '''
                        posmsg.z = StepLength

                        self.PDR.publish(posmsg)

                    valley = False
                    peak = False

                plt.cla()
                plt.subplot(2, 1, 1)
                # plt.plot(signal['signals'], "xr")
                plt.plot(signal, "xr")
                plt.legend(['signals'])
                plt.subplot(2, 1, 2)
                plt.plot(xdata, "ob")
                plt.legend(['pitch'])
                plt.pause(0.001)

        else:
            print("nooooo data ")
            time.sleep(2)

    def shutdown(self):
        rospy.loginfo("Stopping node...")
        rospy.sleep(1)


def thresholding_algo(y, lag, threshold, influence):
    '''
    z-scores smooth
    lag = 5 :for the smoothing functions
    threshold = 3.5 :standard deviations for signal
    influence = 0.5 :between 0 and 1, where 1 is normal influence, 0.5 is half
    '''
    signals = np.zeros(len(y))
    filteredY = np.array(y)
    avgFilter = [0]*len(y)
    stdFilter = [0]*len(y)
    avgFilter[lag - 1] = np.mean(y[0:lag])
    stdFilter[lag - 1] = np.std(y[0:lag])
    for i in range(lag, len(y)):
        if abs(y[i] - avgFilter[i-1]) > threshold * stdFilter[i-1]:
            if y[i] > avgFilter[i-1]:
                signals[i] = 1
            else:
                signals[i] = -1

            filteredY[i] = influence * y[i] + \
                (1 - influence) * filteredY[i-1]
            avgFilter[i] = np.mean(filteredY[(i-lag+1):i+1])
            stdFilter[i] = np.std(filteredY[(i-lag+1):i+1])
        else:
            signals[i] = 0
            filteredY[i] = y[i]
            avgFilter[i] = np.mean(filteredY[(i-lag+1):i+1])
            stdFilter[i] = np.std(filteredY[(i-lag+1):i+1])

    return signals

    '''
    return dict(signals=np.asarray(signals),
                avgFilter=np.asarray(avgFilter),
                stdFilter=np.asarray(stdFilter))
    '''


def PosUpdata(Min_PeakLocation, Min_PkValue, Max_PeakLocation, Max_PkValue, rpy, PositionX, PositionY, a, b, error):
    '''
    Pedestrian Dead Reckoning:
    The data from pitch and yaw show various characteristics in different movement states 
    '''
    pos_start = Min_PeakLocation
    pos_end = Max_PeakLocation

    YawSin = np.median(rpy[pos_start:pos_end])
    YawCos = np.median(rpy[pos_start:pos_end])

    if YawSin > np.pi:
        YawSin = YawSin - (np.pi)

    if YawSin < -np.pi:
        YawSin = YawSin + (np.pi)

    if YawCos > np.pi:
        YawCos = YawCos - (np.pi)

    if YawCos < -np.pi:
        YawCos = YawCos + (np.pi)

    deltaH = np.rad2deg(Max_PkValue - Min_PkValue)
    StepLength = deltaH * a + b
    #StepLength = deltaH*0.027111 + .383

    # position update
    PosX = PositionX + StepLength * \
        np.cos(YawCos-np.deg2rad(error))  # (unit:rad)
    PosY = PositionY + StepLength * np.sin(YawSin-np.deg2rad(error))

    return StepLength, PosX, PosY


def RotMat(PosX, PosY, radians):
    '''
    rotate a point 
    '''

    xx = PosX * np.cos(radians) - PosY * np.sin(radians)
    yy = PosX * np.sin(radians) + PosY * np.cos(radians)

    return xx, yy


if __name__ == '__main__':
    try:

        FINDPEAK()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
