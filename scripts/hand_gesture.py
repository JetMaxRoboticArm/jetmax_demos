#!/usr/bin/env python3
import cv2
import math
import threading
import numpy as np
import rospy
import queue
from sensor_msgs.msg import Image
import hiwonder
from hiwonder import serial_servo as ss
import mediapipe as mp
import simple_ngc

ROS_NODE_NAME = "hand_gesture"
DEFAULT_X, DEFAULT_Y, DEFAULT_Z = 0, 138 + 8.14, 84.4 + 128.4
IMAGE_PIXEL_PRE_100MM_X, IMAGE_PIXEL_PRE_100MM_Y = 260, 280
IMAGE_PROC_SIZE = 640, 480
ORG_PIXEL_X, ORG_PIXEL_Y = 328.3, 380.23

image_queue = queue.Queue(maxsize=1)
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.5)


class HandGesture:
    def __init__(self):
        self.moving_color = None
        self.target_colors = {}
        self.position = DEFAULT_X, DEFAULT_Y, DEFAULT_X
        self.lock = threading.RLock()
        self.position = 0, 0, 0
        self.runner = None
        self.count = 0
        self.gesture_str = ''


ik = hiwonder.kinematic.IKinematic()
state = HandGesture()
ps = (-205, 0 + 10, 150), (-205, 0 + 10, 120), (-205, 0 + 10, 190)


def vector_2d_angle(v1, v2):
    """
       Solve the angle between two vector
    """
    v1_x = v1[0]
    v1_y = v1[1]
    v2_x = v2[0]
    v2_y = v2[1]
    try:
        angle_ = math.degrees(math.acos(
            (v1_x * v2_x + v1_y * v2_y) / (((v1_x ** 2 + v1_y ** 2) ** 0.5) * ((v2_x ** 2 + v2_y ** 2) ** 0.5))))
    except:
        angle_ = 65535.
    if angle_ > 180.:
        angle_ = 65535.
    return angle_


def hand_angle(hand_):
    """
        Obtain the angle of the corresponding hand-related vector, and determine the gesture according to the angle
    """
    angle_list = []
    # ---------------------------- thumb
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[2][0])), (int(hand_[0][1]) - int(hand_[2][1]))),
        ((int(hand_[3][0]) - int(hand_[4][0])), (int(hand_[3][1]) - int(hand_[4][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- index
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[6][0])), (int(hand_[0][1]) - int(hand_[6][1]))),
        ((int(hand_[7][0]) - int(hand_[8][0])), (int(hand_[7][1]) - int(hand_[8][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- middle
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[10][0])), (int(hand_[0][1]) - int(hand_[10][1]))),
        ((int(hand_[11][0]) - int(hand_[12][0])), (int(hand_[11][1]) - int(hand_[12][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- ring
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[14][0])), (int(hand_[0][1]) - int(hand_[14][1]))),
        ((int(hand_[15][0]) - int(hand_[16][0])), (int(hand_[15][1]) - int(hand_[16][1])))
    )
    angle_list.append(angle_)
    # ---------------------------- pink
    angle_ = vector_2d_angle(
        ((int(hand_[0][0]) - int(hand_[18][0])), (int(hand_[0][1]) - int(hand_[18][1]))),
        ((int(hand_[19][0]) - int(hand_[20][0])), (int(hand_[19][1]) - int(hand_[20][1])))
    )
    angle_list.append(angle_)
    return angle_list


def h_gesture(angle_list):
    """
        Use the angle of the corresponding hand-related to define the gesture

    """
    thr_angle = 65.
    thr_angle_thumb = 53.
    thr_angle_s = 49.
    gesture_str = None
    if 65535. not in angle_list:
        if (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "fist"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "hand_heart"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
            gesture_str = "nico-nico-ni"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "hand_heart"
        elif (angle_list[0] > 5) and (angle_list[1] < thr_angle_s) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "one"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] > thr_angle) and (angle_list[4] > thr_angle):
            gesture_str = "two"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] > thr_angle):
            gesture_str = "three"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] > thr_angle) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "three"
        elif (angle_list[0] > thr_angle_thumb) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "four"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] < thr_angle_s) and (angle_list[2] < thr_angle_s) and (
                angle_list[3] < thr_angle_s) and (angle_list[4] < thr_angle_s):
            gesture_str = "five"
        elif (angle_list[0] < thr_angle_s) and (angle_list[1] > thr_angle) and (angle_list[2] > thr_angle) and (
                angle_list[3] > thr_angle) and (angle_list[4] < thr_angle_s):
            gesture_str = "six"
        else:
            "none"
    return gesture_str


ngcs = {
    'one': 'ngcs/1.ngc',
    'two': 'ngcs/2.ngc',
    'three': 'ngcs/3.ngc',
    'four': 'ngcs/4.ngc',
    'five': 'ngcs/5.ngc',
    'six': 'ngcs/6.ngc',
    'hand_heart': 'ngcs/hand_heart.ngc',
}


def move_to_pos(x, y, z, t):
    pos = ik.resolve(x, y, z)
    if pos is None:
        return
    p1, p2, p3 = pos
    if p3 < 490:
        p3 = 490
    ss.set_position(1, int(p1 + 40), t)
    ss.set_position(2, int(p2), t)
    ss.set_position(3, int(p3), t)
    state.position = x, y, z


def draw_num(num):
    if num in ngcs:
        simple_ngc.draw_ngc(ngcs[num])
        move_to_pos(DEFAULT_X, DEFAULT_Y, DEFAULT_Z, 1000)
        rospy.sleep(1.2)


def image_proc():
    img = image_queue.get(block=True)

    if state.runner is not None and state.runner.isAlive():
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        cv2.imshow(ROS_NODE_NAME, img)
        cv2.waitKey(1)
        return

    results = hands.process(img)
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if results.multi_handedness:
        for label in results.multi_handedness:
            pass
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            rospy.logdebug('hand_landmarks:', hand_landmarks)
            mp_drawing.draw_landmarks(img, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            hand_local = []
            for i in range(21):
                x = hand_landmarks.landmark[i].x * img.shape[1]
                y = hand_landmarks.landmark[i].y * img.shape[0]
                hand_local.append((x, y))
            if hand_local:
                angle_list = hand_angle(hand_local)
                gesture_str = h_gesture(angle_list)
                cv2.putText(img, gesture_str, (0, 200), 0, 1.5, (100, 100, 255), 5)
                if gesture_str != state.gesture_str:
                    state.count = 0
                state.gesture_str = gesture_str
                state.count += 1
    else:
        state.count = 0

    if state.count > 5:
        state.count = 0
        state.runner = threading.Thread(target=draw_num, args=(state.gesture_str,), daemon=True)
        state.runner.start()

    cv2.imshow(ROS_NODE_NAME, img)
    cv2.waitKey(1)


def image_callback(ros_image):
    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data)
    image = cv2.flip(image, 1)
    try:
        image_queue.put_nowait(image.copy())
    except queue.Full:
        pass


if __name__ == '__main__':
    rospy.init_node(ROS_NODE_NAME, log_level=rospy.DEBUG)
    rospy.sleep(0.2)
    hiwonder.motor1.set_speed(0)
    hiwonder.motor2.set_speed(100)
    hiwonder.pwm_servo1.set_position(90, 1000)
    move_to_pos(DEFAULT_X, DEFAULT_Y, DEFAULT_Z, 1000)
    rospy.sleep(1)
    hiwonder.motor2.set_speed(0)
    image_sub = rospy.Subscriber('/usb_cam/image_rect_color', Image, image_callback)
    while True:
        try:
            image_proc()
            if rospy.is_shutdown():
                break
        except Exception as e:
            print(e)
            break
