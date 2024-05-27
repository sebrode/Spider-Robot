import rainbow.math.vector3 as V3
import rainbow.simulators.inverse_kinematics.api as IK
import os
import sys
import numpy as np
from scipy.optimize import minimize
import bachelor as Bsc
from adafruit_servokit import ServoKit
from time import sleep
import math


fullAngleMatrix = Bsc.getAngleMatrix()
fullObjectMatrix = Bsc.getObjectMatrix()
step_time = 0.2

kit = ServoKit(channels=16)
leg0 = [0, 1, 2]
leg1 = [4, 5, 6]
leg2 = [8, 9, 10]
leg3 = [12, 13, 14]
legs = [leg0, leg1, leg2, leg3, [leg0, leg1, leg2, leg3]]


def SetAngle(num, ang):
    kit.servo[num].angle = ang


def GetAngle(num):
    return kit.servo[num].angle


allLegs = [legs[0], legs[1], legs[2], legs[3]]

for i in range(max(fullAngleMatrix[0].shape[1], fullAngleMatrix[1].shape[1], fullAngleMatrix[2].shape[1], fullAngleMatrix[3].shape[1])):
    greyAngle1 = GetAngle(int(allLegs[0][0]))
    greyAngle2 = GetAngle(int(allLegs[0][1]))
    greyAngle3 = GetAngle(int(allLegs[0][2]))
    blackAngle1 = GetAngle(int(allLegs[1][0]))
    blackAngle2 = GetAngle(int(allLegs[1][1]))
    blackAngle3 = GetAngle(int(allLegs[1][2]))
    redAngle1 = GetAngle(int(allLegs[2][0]))
    redAngle2 = GetAngle(int(allLegs[2][1]))
    redAngle3 = GetAngle(int(allLegs[2][2]))
    blueAngle1 = GetAngle(int(allLegs[3][0]))
    blueAngle2 = GetAngle(int(allLegs[3][1]))
    blueAngle3 = GetAngle(int(allLegs[3][2]))

    if i < fullAngleMatrix[0].shape[1]:
        greyAngle1 = round(
            90 + math.degrees(float(fullAngleMatrix[0][0][i])), 2)
        greyAngle2 = round(
            90 + math.degrees(float(fullAngleMatrix[0][1][i])), 2)
        greyAngle3 = round(math.degrees(
            abs(float(abs(fullAngleMatrix[0][2][i])))), 2)

    if i < fullAngleMatrix[1].shape[1]:
        blackAngle1 = round(
            90+math.degrees(float(fullAngleMatrix[1][0][i])), 2)
        blackAngle2 = round(
            90+math.degrees(float(fullAngleMatrix[1][1][i])), 2)
        blackAngle3 = round(math.degrees(
            float(abs(fullAngleMatrix[1][2][i]))), 2)

    if i < fullAngleMatrix[2].shape[1]:
        redAngle1 = round(
            90+math.degrees(float(fullAngleMatrix[2][0][i])), 2)
        redAngle2 = round(
            90+math.degrees(float(fullAngleMatrix[2][1][i])), 2)
        redAngle3 = round(math.degrees(
            float(abs(fullAngleMatrix[2][2][i]))), 2)

    if i < fullAngleMatrix[3].shape[1]:
        blueAngle1 = round(
            90+math.degrees(float(fullAngleMatrix[3][0][i])), 2)
        blueAngle2 = round(
            90+math.degrees(float(fullAngleMatrix[3][1][i])), 2)
        blueAngle3 = round(math.degrees(
            float(abs(fullAngleMatrix[3][2][i]))), 2)

    SetAngle(int(allLegs[0][0]), greyAngle1)
    SetAngle(int(allLegs[0][1]), greyAngle2)
    SetAngle(int(allLegs[0][2]), greyAngle3)

    SetAngle(int(allLegs[1][0]), blackAngle1)
    SetAngle(int(allLegs[1][1]), blackAngle2)
    SetAngle(int(allLegs[1][2]), blackAngle3)

    SetAngle(int(allLegs[2][0]), redAngle1)
    SetAngle(int(allLegs[2][1]), redAngle2)
    SetAngle(int(allLegs[2][2]), redAngle3)

    SetAngle(int(allLegs[3][0]), blueAngle1)
    SetAngle(int(allLegs[3][1]), blueAngle2)
    SetAngle(int(allLegs[3][2]), blueAngle3)
    sleep(step_time)
