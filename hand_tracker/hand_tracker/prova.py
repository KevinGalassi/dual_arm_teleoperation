#!/usr/bin/env python3

import mediapipe as mp
import numpy as np
from mediapipe.framework.formats.landmark_pb2 import NormalizedLandmarkList
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np

import cv2 

MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult
from mediapipe import Image
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.components.containers.category import Category
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
mp_drawing_styles = mp.solutions.drawing_styles


def draw_landmarks_on_image(rgb_image, detection_result):
    hand_landmarks_list = detection_result.hand_landmarks
    handedness_list = detection_result.handedness
    annotated_image = np.copy(rgb_image)

    # Loop through the detected hands to visualize.
    for idx in range(len(hand_landmarks_list)):
        hand_landmarks = hand_landmarks_list[idx]
        handedness = handedness_list[idx]

        # Draw the hand landmarks.
        hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
        hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
        ])
        solutions.drawing_utils.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            solutions.hands.HAND_CONNECTIONS,
            solutions.drawing_styles.get_default_hand_landmarks_style(),
            solutions.drawing_styles.get_default_hand_connections_style())

        # Get the top left corner of the detected hand's bounding box.
        height, width, _ = annotated_image.shape
        x_coordinates = [landmark.x for landmark in hand_landmarks]
        y_coordinates = [landmark.y for landmark in hand_landmarks]
        text_x = int(min(x_coordinates) * width)
        text_y = int(min(y_coordinates) * height) - MARGIN

        # Draw handedness (left or right hand) on the image.
        cv2.putText(annotated_image, f"{handedness[0].category_name}",
                    (text_x, text_y), cv2.FONT_HERSHEY_DUPLEX,
                    FONT_SIZE, HANDEDNESS_TEXT_COLOR, FONT_THICKNESS, cv2.LINE_AA)

    return annotated_image


class OAKCamera:

    def __init__(self):
        device_info = dai.Device.getAllAvailableDevices()
        if len(device_info) == 0:
            print('Error with the camera, no device found (Application restarted after is been closed or no camera found )')
            exit()
        device_info = device_info[0]
        self.serial_number = device_info.getMxId()

        pipeline = dai.Pipeline()
            
        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)

        controlIn = pipeline.create(dai.node.XLinkIn)
        videoOut = pipeline.create(dai.node.XLinkOut)
        stillMjpegOut = pipeline.create(dai.node.XLinkOut)

        controlIn.setStreamName("control")
        videoOut.setStreamName("video")
        stillMjpegOut.setStreamName("still")

        # Properties
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_720_P)
        camRgb.setVideoSize(1280, 720)
        camRgb.setFps(30)

        # Linking
        camRgb.video.link(videoOut.input)

        controlIn.out.link(camRgb.inputControl)
        

        self.device = dai.Device(deviceInfo=device_info)
        self.device.startPipeline(pipeline)
        self.qRgb = self.device.getOutputQueue(name="video", maxSize=1, blocking=False)

        self.controlQueue = self.device.getInputQueue("control")

        ctrl = dai.CameraControl()
        ctrl.setAntiBandingMode(dai.CameraControl.AntiBandingMode.OFF)
        ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.AUTO)
        
        #exposure_time, exposure_gain = 1000,1000
        #ctrl.setManualExposure(exposure_time, exposure_gain)
        
        self.controlQueue.send(ctrl)

    def __del__(self):
        cv2.destroyAllWindows()


    def capture(self):
        img = self.qRgb.get().getCvFrame()
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) #!! ACHTUNG !! The output color is in BGR format, is ok for cv2 but not for visualization
        return img
    

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
        hand_gesture_options = vision.GestureRecognizerOptions(    
            base_options=hand_gesture_base_options,
            running_mode=vision.RunningMode.IMAGE,  # or VIDEO for offline
            num_hands=2,
            min_tracking_confidence = 0.3
        )
        
        self.recognizer = vision.GestureRecognizer.create_from_options(hand_gesture_options)


    def detect(self, image:np.array):

        H, W, C = image.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
    
        # landmark detection
        detection_result: HandLandmarkerResult = self.detector.detect(mp_image)


        annotated_image = draw_landmarks_on_image(image, detection_result)

        '''
        
        
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
            l_cropped_np = l_image[:, h_division:]
            l_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(l_cropped_np))

            r_image = mp_image.numpy_view()
            r_cropped_np = r_image[:, :h_division]
            r_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=np.ascontiguousarray(r_cropped_np))

        elif right_hand is None:
            l_cropped_np = mp_image.numpy_view()
            l_mp = mp_image

            width, height = 640, 480
            r_cropped_np = np.zeros((height, width, 3), dtype=np.uint8)
            r_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=r_cropped_np)

        elif left_hand is None:
            r_cropped_np = mp_image.numpy_view()
            r_mp = mp_image

            width, height = 640, 480
            l_cropped_np = np.zeros((height, width, 3), dtype=np.uint8)
            l_mp = mp.Image(image_format=mp.ImageFormat.SRGB, data=l_cropped_np)

        else:
            width, height = 640, 480
            empty_np = np.zeros((height, width, 3), dtype=np.uint8)
            l_cropped_np = empty_np
            r_cropped_np = empty_np
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

        # right
        recognition_result = self.recognizer.recognize(r_mp)
        gestures_list = recognition_result.gestures
        if len(gestures_list) == 0:
            right_gesture = Category(category_name='None', score=1.0)
        else:
            right_gesture = gestures_list[0][0]

        print('Left gesture :', left_gesture.category_name)
        print('Right gesture :', right_gesture.category_name)
        '''
        l_cropped_np = None
        r_cropped_np = None

        recognition_result = self.recognizer.recognize(mp_image)

        #print(recognition_result
        #      )
        for gestures, handedness in zip(recognition_result.gestures, recognition_result.handedness):
            print('gesture:',gestures, 'handness:\n', handedness)

            #if handedness[0].category_name == "Left":
            #    print('Left hand: ', gestures.category_name )
            
            #elif handedness[0].category_name == "Right":
            #    print('right hand: ', gestures.category_name )



        return annotated_image, l_cropped_np, r_cropped_np



#!/usr/bin/env python3

import cv2
import depthai as dai


import cv2
import numpy as np

if __name__ == "__main__":
    # Replace with your actual model paths
    handtracker_task = "/home/kvn/code/sereact_project/src/hand_tracker/task/hand_landmarker.task"
    gesture_task = "/home/kvn/code/sereact_project/src/hand_tracker/task/gesture_recognizer.task"

    detector = Detector(handtracker_task, gesture_task)

    oak = OAKCamera()
    while True:
        image = oak.capture()

        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        ann, left,right = detector.detect(image_rgb)

        cv2.imshow("Camera", ann)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cv2.destroyAllWindows()

