import cv2 as cv
import numpy as np
import math
from math import pi, cos, sin, tan
from enum import Enum

# Output image resolution
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 800

# Image scale (conversion from meters to pixels)
PIXEL_PER_METER = 100

BACKGROUND_COLOR = (255, 255, 255)
LINE_COLOR = (5, 5, 5)

FRAME_DURATION_MS = 0  # (set 0 to wait for a key press)

DT = 0.1 # in seconds

# Dimensions in meters
# World origin is at image center

# Robot dimensions (object A), center is the wheels middle point
WA = 0.4        # Robot width
LA1 = 0.7       # Distance between center and robot front
LA2 = 0.1       # Distance between center and robot back
LA3 = 0.2       # Distance between hitch point and robot back
LA = LA2 + LA3  # Distance betwwen hitch point and robot center

# Trailer dimensions (object B), center is the wheels middle point
WB = 0.4
LB1 = 0.1       # From back to center
LB2 = 0.8       # From center to front
LB3 = 0.2       # From front to hitch point
LB = LB2 + LB3  # From center to hitch point

# Initial pose
THB0 = 0.5
XA0 = 1
YA0 = -2
THA0 = 1.57     # not exactly pi/2 for visual purpose only (no integer pixels)

# Goal
XB_GOAL = -0.1
YB_GOAL = 2
M_GOAL = 0.1    # Margins around trailer

class Phase(Enum):
    GO_FRONTWARD= 0
    TURN_RIGHT = 1
    STOP = 2
    GO_BACKWARD = 3
    PARK_BACKWARD = 4
    PARKED = 5

class State:
    def __init__(self):
        # Minimal state
        self.xA = XA0
        self.yA = YA0
        self.thA = THA0
        self.dthA = 0           # Rotation speed
        self.vA = 0             # Linear speed
        self.dvA = 0            # Linear acceleration
        self.thB = THB0

        # Computed variables
        self.dxA = 0
        self.dyA = 0
        self.xB = 0
        self.yB = 0
        self.dthB = 0

    def update(self):
        self.vA += self.dvA * DT
        self.dxA = self.vA * cos(self.thA)
        self.dyA = self.vA * sin(self.thA)
        self.xA += self.dxA * DT
        self.yA += self.dyA * DT
        self.thA += self.dthA * DT
        dxH = self.dxA + LA * self.dthA * sin(self.thA)
        dyH = self.dyA - LA * self.dthA * cos(self.thA)
        # dxB = dxH + LB * self.dthB * sin(self.thB)
        # dyB = dyH - LB * self.dthB * cos(self.thB)
        self.dthB = (-dxH * sin(self.thB) + dyH * cos(self.thB)) / LB
        self.thB += self.dthB * DT
        self.xB = self.xA - LA * cos(self.thA) - LB * cos(self.thB)
        self.yB = self.yA - LA * sin(self.thA) - LB * sin(self.thB)

def drawLines(lines, x, y, theta):
    rotation = np.array([[cos(theta), sin(theta)], [-sin(theta), cos(theta)]])
    for line in lines:
        points = np.array(line, np.float32)
        pt1 = points[0].T.dot(rotation).T + np.float32((x, y))
        pt2 = points[1].T.dot(rotation).T + np.float32((x, y))
        scale = np.float32((PIXEL_PER_METER, -PIXEL_PER_METER))
        offset = np.float32((IMAGE_WIDTH/2, IMAGE_HEIGHT/2))
        cv.line(
            image,
            np.int32(offset + pt1 * scale),
            np.int32(offset + pt2 * scale),
            LINE_COLOR,
            lineType=cv.LINE_AA,
        )

def drawRobot(x, y, theta):
    ROBOT_LINES = [
        [[-LA2, WA/2], [LA1 - WA/3, WA/2]],
        [[LA1 - WA/3, WA/2], [LA1, 0]],
        [[LA1, 0], [LA1 - WA/3, -WA/2]],
        [[LA1 - WA/3, -WA/2], [-LA2, -WA/2]],
        [[-LA2, -WA/2], [-LA2, WA/2]],
        [[-LA,0], [-LA2,0]],
    ]
    drawLines(ROBOT_LINES, x, y, theta)

def drawTrailer(x, y, theta):
    TRAILER_LINES = [
        [[-LB1, WB/2], [LB2, WB/2]],
        [[LB2, WB/2], [LB2, -WB/2]],
        [[LB2, -WB/2], [-LB1, -WB/2]],
        [[-LB1, -WB/2], [-LB1, WB/2]],
        [[LB2,0], [LB,0]],
    ]
    drawLines(TRAILER_LINES, x, y, theta)

def drawGoal():
    GOAL_LINES = [
        [[-LB1 - M_GOAL, WB/2 + M_GOAL], [LB2, WB/2 + M_GOAL]],
        [[-LB1 - M_GOAL, WB/2 + M_GOAL], [-LB1 - M_GOAL, -WB/2 - M_GOAL]],
        [[-LB1 - M_GOAL, -WB/2 - M_GOAL], [LB2, -WB/2 - M_GOAL]],
    ]
    drawLines(GOAL_LINES, XB_GOAL, YB_GOAL, 0)

def clearImage():
    cv.rectangle(image, (0, 0), (IMAGE_WIDTH, IMAGE_HEIGHT), BACKGROUND_COLOR, cv.FILLED)

def printObj(obj):
    for key, value in vars(obj).items():
        print(f"{key}: {value:.3f}")

def printText(text, line):
    cv.putText(image, text, (15, 30+line*35), cv.FONT_HERSHEY_SIMPLEX, 1, LINE_COLOR, 1)

def printInfo(phase, time, state):
    printText(f"phase: {phase.name}",0)
    printText(f"time: {time:.1f} s",2)
    line = 4
    for key, value in vars(state).items():
        printText(f"{key}: {value:.3f}", line)
        line += 1

image = np.zeros(shape=[IMAGE_HEIGHT, IMAGE_WIDTH, 3], dtype=np.uint8)
phase = Phase.GO_FRONTWARD
state = State()

for i in range(1000):
    clearImage()
    drawGoal()

    if phase==Phase.GO_FRONTWARD:
        state.vA = 1
        if state.yA + LA1 > YB_GOAL :
            phase = Phase.TURN_RIGHT
    if phase == Phase.TURN_RIGHT:
        state.dthA = -0.8
        if state.thA < 0:
            phase = Phase.STOP
    if phase == Phase.STOP:
        state.dvA = -0.6
        state.dthA = 0
        if state.vA < 0:
            phase = Phase.GO_BACKWARD
    if phase == Phase.GO_BACKWARD:
        state.dvA = -0.4
        state.dthA = 0
        if state.vA < -0.2:
            phase = Phase.PARK_BACKWARD
    if phase == Phase.PARK_BACKWARD:
        # TODO : control law
        state.dvA = 0
        if state.xA < XB_GOAL:
            phase = Phase.PARKED
    if phase == Phase.PARKED:
        state.dthA = 0
        state.vA = 0
        state.dvA = 0

    state.update()
    drawRobot(state.xA, state.yA, state.thA)
    drawTrailer(state.xB, state.yB, state.thB)
    printInfo(phase, i*DT, state)

    cv.imwrite(f"output/img{i:03d}.png",image)
    cv.imshow("differential drive robot", image)

    # Wait and test escape key
    if cv.waitKey(FRAME_DURATION_MS) == 27:
        exit(1)
