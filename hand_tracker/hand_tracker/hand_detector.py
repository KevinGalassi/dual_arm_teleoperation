#!/usr/bin/env python3

import mediapipe as mp
import numpy as np

from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult
from mediapipe import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.components.containers.category import Category

class Detector:
    
    def __init__(self, 
            handtracker_task:str,
            gesture_task:str,
            
    ):

        hand_landmark_base_options = python.BaseOptions(model_asset_path=handtracker_task)
        hand_landmark_options = vision.HandLandmarkerOptions(base_options=hand_landmark_base_options,
                                            num_hands=2)
        self.detector = vision.HandLandmarker.create_from_options(hand_landmark_options)

        hand_gesture_base_options = python.BaseOptions(model_asset_path=gesture_task)
        hand_gesture_options = vision.GestureRecognizerOptions(base_options=hand_gesture_base_options)
        self.recognizer = vision.GestureRecognizer.create_from_options(hand_gesture_options)

    def detect(self, image:np.array):

        H, W, C = image.shape
        mp_image = Image(image_format=mp.ImageFormat.SRGB, data=image)
    
        # landmark detection
        detection_result: HandLandmarkerResult = self.detector.detect(mp_image)

        left_hand, right_hand = None, None
        for landmarks, handedness in zip(
            detection_result.hand_landmarks, detection_result.handedness
        ):
            if handedness[0].category_name == "Left":
                left_hand = landmarks
            elif handedness[0].category_name == "Right":
                right_hand = landmarks

        if left_hand and right_hand:
            # TODO: Assuming that the palm are facing the camera. 
            x, y, z = zip(*[(landmark.x, landmark.y, landmark.z) for landmark in left_hand])
            left_bound = int(min(x) * W)

            x, y, z = zip(*[(landmark.x, landmark.y, landmark.z) for landmark in right_hand])
            right_bound = int(max(x) * W)

            h_division = (left_bound + right_bound) // 2

            l_image = mp_image.numpy_view()
            cropped_np = l_image[:, h_division:]
            l_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(cropped_np))

            r_image = mp_image.numpy_view()
            cropped_np = l_image[:, :h_division]
            r_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(cropped_np))

        elif right_hand is None:
            l_mp = mp_image
            width, height = 640, 480
            empty_np = np.zeros((height, width, 3), dtype=np.uint8)
            r_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=empty_np)

        elif left_hand is None:
            r_mp = mp_image
            width, height = 640, 480
            empty_np = np.zeros((height, width, 3), dtype=np.uint8)
            l_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=empty_np)

        else:
            width, height = 640, 480
            empty_np = np.zeros((height, width, 3), dtype=np.uint8)
            l_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=empty_np)
            r_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=empty_np)

        # gesture recognition
        # left
        recognition_result = self.recognizer.recognize(l_mp)
        gestures_list = recognition_result.gestures
        if len(gestures_list) == 0:
            left_gesture = Category(category_name='None', score=1.0)
        else:
            left_gesture = gestures_list[0][0]
        print('Left gesture :', left_gesture.category_name)

        # right
        recognition_result = self.recognizer.recognize(r_mp)
        gestures_list = recognition_result.gestures
        if len(gestures_list) == 0:
            right_gesture = Category(category_name='None', score=1.0)
        else:
            right_gesture = gestures_list[0][0]

        return 