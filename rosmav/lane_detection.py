import cv2
import numpy as np
import matplotlib.pyplot as plt
from random import randrange




def detect_lines(img, threshold1 = 50, threshold2 = 150, apertureSize = 3,  minLineLength = 100, maxLineGap = 10):

    # img = cv2.imread(img)
    img = cv2.resize(img, (1100, 650))
    height, width = img.shape[:2]
    img = img[height//3: :]  # This slices the top half of the image

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # convert to grayscale
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertureSize) # detect edges
    lines = cv2.HoughLinesP(
                    edges,
                    1,
                    np.pi/180,
                    100,
                    minLineLength=minLineLength,
                    maxLineGap=maxLineGap,
            ) # detect lines

    if lines is not None: 
        print("NUM LINES", len(lines))
    return img, lines


def draw_lines(img, lines, color = (255, 0, 255)):
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (x1, y1), (x2, y2), (randrange(255),randrange(255),randrange(255)), 2)

    plt.imshow(img)

def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    for line in lines:
        
        print(line[0])
        x1 = line[0][0]
        y1 = line[0][1]
        x2 = line[0][2]
        y2 = line[0][3]


        if x2 - x1 == 0:
            denominator = 0.000001
        else:
            denominator = x2 - x1


        slope = float((y2-y1)/(denominator))

        if slope == 0:
            slope == 0.000001

        intercept = float(x2-y2/slope)
        slopes.append(slope)
        intercepts.append(intercept)
    print (slopes)
    print (intercepts) 
    return slopes, intercepts


def detect_lanes(lines):
    slopes, intercepts = get_slopes_intercepts(lines)
    pairs = []
    EPSILON = 4

    for i in range(len(slopes)):
        for j in range(i + 1, len(slopes)):
            if abs(slopes[i] - slopes[j]) <= EPSILON:
                pairs.append([lines[i], lines[j]])

    return pairs


def draw_lanes(img, lanes):
    if len(lanes) <= 0:
        pass
    else:
        for lane in lanes:
            if len(lane) == 2:
                line1, line2 = lane
                x1, y1, x2, y2 = line1[0]
                x3, y3, x4, y4 = line2[0]
                cv2.line(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
                cv2.line(img, (x3, y3), (x4, y4), (255, 0, 0), 2)
    plt.imshow(img)




def return_slopes(image):
    img99, lines99 = detect_lines(image, 100, 105, 3, 100, 1800)
    slopes99, intercepts99 = get_slopes_intercepts(lines99)


    return slopes99
