#coding=utf-8
import cv2
import numpy as np

def get_svm(path):
    svm = cv2.ml_SVM.load(path)
    return svm

def find_arrow(red_contours, green_contours, frame, arrow_svm):
    color = ""
    contour = []
    arrow_box = []
    arrow_img = []

    for red_contour in red_contours:
        if cv2.contourArea(red_contour) < 2000:
            break
        ellipse = cv2.fitEllipse(red_contour)
        rect = cv2.minAreaRect(cv2.boxPoints(ellipse))
        box = np.int0(cv2.boxPoints(rect))

        min_index = box.min(0)
        max_index = box.max(0)
        img = frame[min_index[1]:max_index[1], min_index[0]:max_index[0]]
        shape = np.shape(img)

        if shape[0] == 0 or shape[1] == 0 or arrow_judge(img, arrow_svm) is False:
            continue

        color = "red"
        contour = red_contour
        arrow_box = box
        arrow_img = img
        break

    for green_contour in green_contours:
        if cv2.contourArea(green_contour) < 2000:
            break
        ellipse = cv2.fitEllipse(green_contour)
        rect = cv2.minAreaRect(cv2.boxPoints(ellipse))
        box = np.int0(cv2.boxPoints(rect))

        min_index = box.min(0)
        max_index = box.max(0)
        img = frame[min_index[1]:max_index[1], min_index[0]:max_index[0]]
        shape = np.shape(img)

        if shape[0] == 0 or shape[1] == 0 or arrow_judge(img, arrow_svm) is False:
            continue
        color = "green"
        contour = green_contour
        arrow_box = box
        arrow_img = img
        break

    return color, contour, arrow_box, arrow_img

def arrow_judge(img, svm):
    img = cv2.resize(img, (64, 64))
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, gray_img = cv2.threshold(gray_img, 0, 255, cv2.THRESH_OTSU)
    vector = np.ravel(gray_img).astype(np.float32)
    vector = np.resize(vector, (1, 64*64))
    _, result = svm.predict(vector)
    if result[0] > 0:
        return True
    else:
        return False

def get_distance(point1, point2):
    distance = pow( pow( (point1[0]-point2[0]), 2) + pow( (point1[1]-point2[1]), 2), 0.5 )
    return distance

def get_line(rect_point):
    point1 = rect_point[0]
    if get_distance(rect_point[0], rect_point[1]) > get_distance(rect_point[0], rect_point[3]):
        point2 = rect_point[3]
    else:
        point2 = rect_point[1]

    k = ( point2[0] - point1[0] ) / ( point2[1] - point1[1] )
    point_sum = rect_point.sum(axis=0)
    center = (int(point_sum[0] / 4), int(point_sum[1] / 4))

    b = center[0] - center[1] * k
    return k, b

def get_direction(contour, k, b):
    left_count = 0
    right_count = 0

    for point in contour:
        if point[0][1] * k + b < point[0][0]:
            right_count += 1
        else:
            left_count += 1
    # print("left", left_count, right_count)
    if left_count > right_count:
        return "left"
    else:
        return "right"