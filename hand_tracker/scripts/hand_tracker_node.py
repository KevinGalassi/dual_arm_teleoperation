#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


from hand_tracker.msg import HandTrack
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header


from ament_index_python.packages import get_package_share_directory
import os
from cv_bridge import CvBridge

#!/usr/bin/env python3

import mediapipe as mp
import numpy as np

from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult


from mediapipe.tasks.python.vision.gesture_recognizer import GestureRecognizerResult



from mediapipe.tasks import python
from mediapipe.tasks.python import vision
from mediapipe.tasks.python.components.containers.category import Category
from mediapipe.tasks.python.components.containers.landmark import NormalizedLandmark



from mediapipe.framework.formats.landmark_pb2 import NormalizedLandmarkList
from mediapipe.tasks.python.vision.hand_landmarker import HandLandmarkerResult

from mediapipe import solutions
from mediapipe.framework.formats import landmark_pb2
import numpy as np

import cv2 
import math


MARGIN = 10  # pixels
FONT_SIZE = 1
FONT_THICKNESS = 1
HANDEDNESS_TEXT_COLOR = (88, 205, 54) # vibrant green

mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands
mp_drawing_styles = mp.solutions.drawing_styles

# https://stackoverflow.com/questions/72003980/can-we-get-the-orientation-of-the-hand-from-mediapipes-palm-detector


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


def display_batch_of_images_with_gestures_and_hand_landmarks(images, recognition_result):

    """Displays a batch of images with the gesture category and its score along with the hand landmarks."""
    # Images and labels.
    

    results = []

    top_gesture = recognition_result.gestures[0][0]
    hand_landmarks = recognition_result.hand_landmarks

    images = [images]
    results = [(top_gesture, hand_landmarks)]
 
    images = [image.numpy_view() for image in images]
    gestures = [top_gesture for (top_gesture, _) in results]
    multi_hand_landmarks_list = [multi_hand_landmarks for (_, multi_hand_landmarks) in results]

    # Auto-squaring: this will drop data that does not fit into square or square-ish rectangle.
    rows = int(math.sqrt(len(images)))
    cols = len(images) // rows

    # Size and spacing.
    FIGSIZE = 13.0
    SPACING = 0.1


    # Display gestures and hand landmarks.
    for i, (image, gestures) in enumerate(zip(images[:rows*cols], gestures[:rows*cols])):
        title = f"{gestures.category_name} ({gestures.score:.2f})"
        dynamic_titlesize = FIGSIZE*SPACING/max(rows,cols) * 40 + 3
        annotated_image = image.copy()

        for hand_landmarks in multi_hand_landmarks_list[i]:
          hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
          hand_landmarks_proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmarks
          ])

          mp_drawing.draw_landmarks(
            annotated_image,
            hand_landmarks_proto,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing_styles.get_default_hand_landmarks_style(),
            mp_drawing_styles.get_default_hand_connections_style())
    return annotated_image


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
            num_hands=2,
            min_hand_detection_confidence=0.5,   # higher threshold filters weak detections
            min_hand_presence_confidence=0.3,    # ensures stable presence classification
            min_tracking_confidence=0.5          # more consistent across frames
        )

        self.recognizer = vision.GestureRecognizer.create_from_options(hand_gesture_options)


    def draw_image(self, image : np.array, hand_landmarks:list)-> np.array:

        for hand_landmark in hand_landmarks:
            hand_landmarks_proto = landmark_pb2.NormalizedLandmarkList()
            hand_landmarks_proto.landmark.extend([
                landmark_pb2.NormalizedLandmark(x=landmark.x, y=landmark.y, z=landmark.z) for landmark in hand_landmark
            ])

            mp_drawing.draw_landmarks(
                image,
                hand_landmarks_proto,
                mp_hands.HAND_CONNECTIONS,
                mp_drawing_styles.get_default_hand_landmarks_style(),
                mp_drawing_styles.get_default_hand_connections_style())
        
        return image           
    
    def gesture_to_int(self, gesture: str) -> int:
        if gesture == 'Thumb_Up':
            return HandTrack.THUMB_UP
        elif gesture == 'Thumb_Down':
            return HandTrack.THUMB_DOWN
        elif gesture == 'Victory':
            return HandTrack.VICTORY
        elif gesture == 'Pointing_Up':
            return HandTrack.POINTING_UP
        elif gesture == 'Closed_Fist':
            return HandTrack.CLOSED_FIST
        elif gesture == 'Open_Palm':
            return HandTrack.OPEN_PALM
        elif gesture == 'ILoveYou':
            return HandTrack.ILOVEU
        
        return HandTrack.NONE

    def detection_to_msg(self, handedness:Category, landmarks:list[NormalizedLandmark], gesture:Category, image:np.array) -> HandTrack:

        msg = HandTrack()

        hand : Category = handedness[0]
        landmarks : list[NormalizedLandmark] = landmarks
        gesture : Category = gesture[0]

        # Landmark order from: https://ai.google.dev/static/mediapipe/images/solutions/hand-landmarks.png?hl=it
        # (0:0, 5:1, 17:2 )
        points = np.array([
            [landmarks[0].x, landmarks[0].y, landmarks[0].z],
            [landmarks[5].x, landmarks[5].y, landmarks[5].z],
            [landmarks[17].x, landmarks[17].y, landmarks[17].z],
        ]) 

        # Pose
        pose = np.mean(points, axis=0)

        if hand.category_name == 'Left':
            pose[0] = pose[0] - 0.5

        # NOTE
        # Required adjustment to make the values in range [-1,1] and compelling with world coordinates
        msg.point.x = np.clip(1 - 2*pose[1], -1, 1)
        msg.point.y = np.clip(-(4 * pose[0] -1), -1, 1)    
        msg.point.z = np.clip(pose[2], -1, 1)



        # Angle
        p1 = np.array([landmarks[0].x, landmarks[0].y, landmarks[0].z])
        p2 = np.array([landmarks[13].x, landmarks[13].y, landmarks[13].z])

        v1 = np.array([1,0,0])
        v2 = (p2 - p1)

        v1, v2 = v1[:2], v2[:2]
        dot = np.dot(v1, v2)
        norms = np.linalg.norm(v1) * np.linalg.norm(v2)
        angle = np.arccos(dot / norms)
        angle = -angle + np.pi/2 
        msg.angle = angle

        # Gesture
        msg.gesture_id = self.gesture_to_int(gesture.category_name)

        # Image dimension
        H, W, _ = image.shape
        msg.image_width = W
        msg.image_height = H

        return msg


    def detect(self, image:np.array):

        H, W, C = image.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
    
        res : GestureRecognizerResult = self.recognizer.recognize(mp_image)
        # res : 
        # res.handedness : List[List[Category]]: For each hand, return a list of categoroies detected, 0 is the top recognized
        # res.hand_landmarks : List[List[NormalizedLandmark]]: for each hand, return a list of len=21 of Normalizedmark
        # res.gestures:  List[List[Category]]: For each hand, return a list of categories detected, 0 is the top recognized
        #

        left_msg, right_msg = HandTrack(), HandTrack()

        for handedness, hand_landmarks, gesture in zip(res.handedness, res.hand_landmarks, res.gestures):
            if handedness[0].display_name == 'Left':
                left_msg = self.detection_to_msg(handedness, hand_landmarks, gesture, image)
            elif handedness[0].display_name == 'Right':
                right_msg = self.detection_to_msg(handedness, hand_landmarks, gesture, image)
            else:
                pass

            image = self.draw_image(image, res.hand_landmarks)
        
        # NOTE
        # Added so that user can see left hand on the left of the screen and right hand on the right of the screen
        # If added in the image callback, the gesture recognize detect the left hand as right and viceversa. 
        image = cv2.flip(image, flipCode=1)
        image = cv2.line(image, (W//2,0), (W//2,H),(255, 0, 0), 2)
        return image, left_msg, right_msg



class HandTracker(Node):


    def __init__(self):
        super().__init__('minimal_publisher')

        package_name = 'hand_tracker'
        package_path = get_package_share_directory(package_name)
        folder_path = os.path.join(package_path, 'task')
        gesture_path = os.path.join(folder_path, 'gesture_recognizer.task')
        tracker_path = os.path.join(folder_path, 'hand_landmarker.task')


        CAMERA_TOPIC = '/image_raw'

        self.detector = Detector(tracker_path, gesture_path)

        self.pub_image = self.create_publisher(Image, '/image_results', 10)
        self.pub_l_track = self.create_publisher(HandTrack, '/left/hand_tracking', 10)
        self.pub_r_track = self.create_publisher(HandTrack, '/right/hand_tracking', 10)


        self.sub_image_ = self.create_subscription(Image, CAMERA_TOPIC, self.image_callback, 10)
        self.current_image = None

        self.cv_bridge = CvBridge()

        timer_frequency = 240 # Hz
        timer_period = 1.0/timer_frequency  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def image_callback(self, msg:Image):
        self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def timer_callback(self):
        
        if not isinstance(self.current_image, type(None)):

            header = Header()
            header.stamp = self.get_clock().now().to_msg()

            detection_image, left_msg, right_msg = self.detector.detect(self.current_image.copy())
            
            left_msg.header = header
            right_msg.header = header

            an_msg = self.cv_bridge.cv2_to_imgmsg(detection_image, encoding="rgb8")
            self.pub_image.publish(an_msg)
            
            self.pub_l_track.publish(left_msg)
            self.pub_r_track.publish(right_msg)


        self.current_image = None





def main(args=None):

    rclpy.init(args=args)

    hand_tracker = HandTracker()


    
    while rclpy.ok() and not isinstance(hand_tracker.current_image, type(None)):
        rclpy.spin_once(hand_tracker, timeout_sec=0.1)


    rclpy.spin(hand_tracker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    hand_tracker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()