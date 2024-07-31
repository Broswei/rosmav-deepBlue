import cv2
import numpy as np


def detect_lines(
    img, threshold1=50, threshold2=150, apertureSize=3, minLineLength=100, maxLineGap=10
):
    img = cv2.resize(img, (640, 480))

    height, width = img.shape[:2]
    img = img[height//2: :]  # This slices the top half of the image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, threshold1, threshold2, apertureSize=apertureSize)
    lines = cv2.HoughLinesP(
        edges, 1, np.pi / 180, 100, minLineLength=minLineLength, maxLineGap=maxLineGap
    )
    return img, lines


def draw_lines(img, lines, color=(0, 255, 0)):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), color, 2)
    return img


def get_slopes_intercepts(lines):
    slopes = []
    intercepts = []
    if lines is not None:
        for line in lines:
            # x1, y1, x2, y2 = line[0]
            x1 = line[0][0]
            y1 = line[0][1]
            x2 = line[0][2]
            y2 = line[0][3]

            slope = (y2 - y1) / (x2 - x1) if x2 != x1 else np.inf
            intercept = y1 - slope * x1
            slopes.append(slope)
            intercepts.append(intercept)
    return slopes, intercepts


# def detect_lanes(lines):
#     slopes, intercepts = get_slopes_intercepts(lines)
#     lanes = []
#     if lines is not None:
#         for i in range(len(lines)):
#             for j in range(i + 1, len(lines)):
#                 if slopes[i] == slopes[j] and abs(intercepts[i] - intercepts[j]) < 50:
#                     lanes.append([lines[i], lines[j]])
#     return lanes


# def draw_lanes(img, lanes):
#     colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
#     for i, lane in enumerate(lanes):
#         color = colors[i % len(colors)]
#         for line in lane:
#             x1, y1, x2, y2 = line[0]
#             cv2.line(img, (x1, y1), (x2, y2), color, 2)
#     return img