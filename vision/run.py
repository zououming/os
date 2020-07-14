#coding=utf-8
import cv2
import sys
import time
import os
import numpy as np
import serial_arduino as ser
import adjust_hsv
import judge

def set_parameter():
    global hsv_flag, xml_path, collect_flag, collect_time
    model = "car"
    frame_threshold = 6
    if len(sys.argv) > 1:
        for i in range(len(sys.argv)):
            if sys.argv[i].startswith("-"):
                option = sys.argv[i][1:]
                if option == "frame":
                    frame_threshold = int(sys.argv[i+1])
                elif option == "model":
                    model = sys.argv[i+1]
                elif option == "collect":
                    collect_flag = 1
                elif option == "hsv":
                    hsv_flag = True
                    # xml_path = sys.argv[i+1]
    return model, frame_threshold

def get_cnt_area(contour):
    return cv2.contourArea(contour)

def test(frame_threshold):
    global green_lower, green_upper, red_lower, red_upper, collect_flag, collect_time
    frame_count = 0
    collect_count = 0
    last_time = time.time()
    color_direction_count = np.zeros(4)
    color_direction_map = ["green left", "green right", "red left", "red right"]
    capture = cv2.VideoCapture(0)
    arrow_svm = judge.get_svm("setting/arrow.xml")
    if collect_flag:
        os.system("mkdir img")

    while(capture.isOpened()):
        ret, frame = capture.read()

        if collect_flag and time.time() - last_time >= collect_time:
            img_path = "img/" + str(collect_count) + ".jpg"
            cv2.imwrite(img_path, frame)
            print("collect:",img_path)
            collect_count += 1
            last_time = time.time()

        cv2.imshow("frame", frame)
        key = cv2.waitKey(1)
        if key == 27:
            break

        frame_count += 1
        if frame_count > frame_threshold:
            print("No arrow!")
            frame_count = 0
            for i in range(4):
                color_direction_count[i] = 0

        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_green = cv2.inRange(hsv_img, (green_lower[0], green_lower[1], green_lower[2]),
                                (green_upper[0], green_upper[1], green_upper[2]))

        hsv_red1 = cv2.inRange(hsv_img, (red_lower[0], red_lower[2], red_lower[3]),
                               (red_upper[0], red_upper[2], red_upper[3]))
        hsv_red2 = cv2.inRange(hsv_img, (red_lower[1], red_lower[2], red_lower[3]),
                               (red_upper[1], red_upper[2], red_upper[3]))
        hsv_red = cv2.bitwise_or(hsv_red1, hsv_red2)

        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        # kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))

        # hsv_red = cv2.erode(hsv_red, kernel)
        # hsv_green = cv2.erode(hsv_green, kernel)
        #
        # hsv_red = cv2.dilate(hsv_red, kernel2)
        # hsv_green = cv2.dilate(hsv_green, kernel2)

        cv2.imshow("red", hsv_red)
        cv2.imshow("green", hsv_green)

        red_contours, _ = cv2.findContours(hsv_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        green_contours, _ = cv2.findContours(hsv_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        red_contours.sort(key=get_cnt_area, reverse=True)
        green_contours.sort(key=get_cnt_area, reverse=True)

        color, contour, box, arrow_img = judge.find_arrow(red_contours, green_contours, frame, arrow_svm)

        if len(contour) == 0:
            continue

        cv2.imshow("arrow", arrow_img)

        k, b = judge.get_line(box)
        direction = judge.get_direction(contour, k, b)

        index = 0
        if color == "red":
            index += 2
        if direction == "right":
            index += 1

        color_direction_count[index] += 1

        if sum(color_direction_count) >= frame_threshold / 2:
            frame_count = 0
            max_index = np.argmax(color_direction_count)
            print(max_index)
            print(color_direction_map[max_index])
            for i in range(4):
                color_direction_count[i] = 0

        cv2.waitKey(1)

def car(frame_threshold, serial):
    global green_lower, green_upper, red_lower, red_upper, collect_flag, collect_time
    collect_count = 0
    color_direction_count = np.zeros(4)
    color_direction_map = ["green left", "green right", "red left", "red right"]

    capture = cv2.VideoCapture(0)
    arrow_svm = judge.get_svm("setting/arrow.xml")

    if collect_flag:
        os.system("mkdir img")
        os.system("rm -r img/*")

    frame_count = 0
    while(capture.isOpened()):
        if sum(color_direction_count) > frame_threshold:
            max_index = np.argmax(color_direction_count)
            # if ser.serial_read(serial):
            ser.serial_send(serial, str(max_index))
            cv2.waitKey(1000)
            if ser.serial_read(serial) or max_index >= 2:
                for i in range(4):
                    color_direction_count[i] = 0

        ret, frame = capture.read()

        if frame_count > frame_threshold:
            frame_count = 0
            print("no arrow!")
            for i in range(4):
                color_direction_count[i] = 0
        # hsv_img = cv2.resize(frame, (200, 150))
        hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        hsv_green = cv2.inRange(hsv_img, (green_lower[0], green_lower[1], green_lower[2]),
                                (green_upper[0], green_upper[1], green_upper[2]))

        hsv_red1 = cv2.inRange(hsv_img, (red_lower[0], red_lower[2], red_lower[3]),
                               (red_upper[0], red_upper[2], red_upper[3]))
        hsv_red2 = cv2.inRange(hsv_img, (red_lower[1], red_lower[2], red_lower[3]),
                               (red_upper[1], red_upper[2], red_upper[3]))
        hsv_red = cv2.bitwise_or(hsv_red1, hsv_red2)

        # kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        # kernel2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        #
        # hsv_red = cv2.erode(hsv_red, kernel)
        # hsv_green = cv2.erode(hsv_green, kernel)
        #
        # hsv_red = cv2.dilate(hsv_red, kernel2)
        # hsv_green = cv2.dilate(hsv_green, kernel2)

        red_contours, _ = cv2.findContours(hsv_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        green_contours, _ = cv2.findContours(hsv_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

        red_contours.sort(key=get_cnt_area, reverse=True)
        green_contours.sort(key=get_cnt_area, reverse=True)

        color, contour, box, _ = judge.find_arrow(red_contours, green_contours, frame, arrow_svm)

        if len(contour) == 0:
            continue

        k, b = judge.get_line(box)
        direction = judge.get_direction(contour, k, b)

        index = 0
        if color == "red":
            index += 2
        if direction == "right":
            index += 1

        color_direction_count[index] += 1

        if collect_flag:
            max_index = np.argmax(color_direction_count)
            img_path = "img/" + color_direction_map[max_index] + str(collect_count) + ".jpg"
            collect_count += 1
            cv2.imwrite(img_path, frame)

        # if sum(color_direction_count) > frame_threshold:
        #     max_index = np.argmax(color_direction_count)
        #     # if ser.serial_read(serial):
        #     ser.serial_send(serial, str(max_index))
        #     if ser.serial_read(serial):
        #         for i in range(4):
        #             color_direction_count[i] = 0

        max_index = np.argmax(color_direction_count)
        print(color_direction_map[max_index])
        cv2.waitKey(1)

green_lower = [35, 43, 36]
green_upper = [77, 255, 255]
red_lower = [0, 156, 43, 46]
red_upper = [10, 180, 255, 255]
hsv_flag = False
xml_path = "./setting/hsv.xml"
collect_flag = False

if __name__ == "__main__":
    model, frame_threshold = set_parameter()
    print("model:", model)
    print("frame threshold:", frame_threshold)

    if hsv_flag:
        print("read xml file:", xml_path)
        green_lower, green_upper, red_lower, red_upper = adjust_hsv.read_xml(xml_path)
        green_lower, green_upper, red_lower, red_upper = adjust_hsv.to_list(green_lower, green_upper, red_lower, red_upper)

    print("green lower:", green_lower)
    print("green upper:", green_upper)
    print("red lower:", red_lower)
    print("red upper:", red_upper)

    if collect_flag:
        print("collect: on")

    if model == "test":
        test(frame_threshold)
    elif model == "car":
        serial = ser.serial_init("/dev/ttyACM0")
        car(frame_threshold, serial)