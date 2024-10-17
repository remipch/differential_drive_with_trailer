import cv2 as cv
import numpy as np
import math

# Output image resolution
IMAGE_WIDTH = 1000
IMAGE_HEIGHT = 800

# Image scale (conversion from meters to pixels)
PIXEL_PER_METER = 50

BACKGROUND_COLOR = (255, 255, 255)
LINE_COLOR = (5, 5, 5)

FRAME_DURATION_MS = 0  # (set 0 to wait for a key press)

DT = 0.1 # in seconds

# Dimensions in meters

# Robot (object A), center is the wheels middle point
WA = 0.4        # Robot width
LA1 = 0.7       # Distance between center and robot front
LA2 = 0.1       # Distance between center and robot back
LA3 = 0.1       # Distance between hitch point and robot back
LA = LA2 + LA3  # Distance betwwen hitch point and robot center


def drawLines(lines, x, y, theta):
    rotation = np.array([[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]])
    for line in lines:
        points = np.array(line, np.float32)
        pt1 = points[0].T.dot(rotation).T + np.float32((x, y))
        pt2 = points[1].T.dot(rotation).T + np.float32((x, y))
        pt1 *= np.float32((PIXEL_PER_METER, PIXEL_PER_METER))
        pt2 *= np.float32((PIXEL_PER_METER, PIXEL_PER_METER))
        pt1[1] = IMAGE_HEIGHT - pt1[1]
        pt2[1] = IMAGE_HEIGHT - pt2[1]
        cv.line(
            image,
            np.int32(pt1),
            np.int32(pt2),
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


image = np.zeros(shape=[IMAGE_HEIGHT, IMAGE_WIDTH, 3], dtype=np.uint8)

for i in range(100):
    t = i * DT

    # Clear image
    cv.rectangle(image, (0, 0), (IMAGE_WIDTH, IMAGE_HEIGHT), BACKGROUND_COLOR, cv.FILLED)

    drawRobot(1+t,1, i/50)

    cv.imwrite(
        f"output/img{i:03d}.png",
        image,
    )
    cv.imshow("differential drive robot", image)

    # Wait and test escape key
    if cv.waitKey(FRAME_DURATION_MS) == 27:
        exit(1)
