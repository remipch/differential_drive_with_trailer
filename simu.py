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

DT = 0.1 # in second

E = 0.4
LA1 = 0.7
LA2 = 0.1
LA3 = 0.1
LA = LA2 + LA3

ROBOT_LINES = [
    [[-LA2, E/2], [LA1 - E/3, E/2]],
    [[LA1 - E/3, E/2], [LA1, 0]],
    [[LA1, 0], [LA1 - E/3, -E/2]],
    [[LA1 - E/3, -E/2], [-LA2, -E/2]],
    [[-LA2, -E/2], [-LA2, E/2]],
]

def drawLines(lines, x, y, theta):
    rotation = np.array([[math.cos(theta), math.sin(theta)], [-math.sin(theta), math.cos(theta)]])
    for line in lines:
        # print(f"line {line}", flush=True)
        points = np.array(line, np.float32)
        pt1 = points[0].T.dot(rotation).T + np.float32((x, y))
        pt2 = points[1].T.dot(rotation).T + np.float32((x, y))
        pt1 *= np.float32((PIXEL_PER_METER, PIXEL_PER_METER))
        pt2 *= np.float32((PIXEL_PER_METER, PIXEL_PER_METER))
        pt1[1] = IMAGE_HEIGHT - pt1[1]
        pt2[1] = IMAGE_HEIGHT - pt2[1]
        # print(f"pt1: {pt1}", flush=True)
        # print(f"pt2: {pt2}", flush=True)
        cv.line(
            image,
            np.int32(pt1),
            np.int32(pt2),
            LINE_COLOR,
            lineType=cv.LINE_AA,
        )

def drawRobot(x, y, theta):
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
