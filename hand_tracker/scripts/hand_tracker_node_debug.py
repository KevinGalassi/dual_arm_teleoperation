#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from hand_tracker.msg import HandTrack
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
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


    def detect(self, image:np.array):

        H, W, C = image.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
    
        # landmark detection
        detection_result: HandLandmarkerResult = self.detector.detect(mp_image)


        annotated_image = draw_landmarks_on_image(image, detection_result)

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
        recognition_result : GestureRecognizerResult = self.recognizer.recognize(l_mp)
        print(type(recognition_result))
        #print(len(recognition_result.hand_landmarks))
        
        if len(recognition_result.hand_landmarks) > 0:
            for i in range(len(recognition_result.hand_landmarks)):
                print(f'{i} : {len(recognition_result.hand_landmarks[0])}')

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

        return annotated_image, l_cropped_np, r_cropped_np



    def single_detect(self, image:np.array):

        H, W, C = image.shape
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        #mp_image = mp.Image(data=image)
    
        # landmark detection
        detection_result: HandLandmarkerResult = self.detector.detect(mp_image)



        #annotated_image = draw_landmarks_on_image(image, detection_result)
     
        # gesture recognition
        # left
        res : GestureRecognizerResult = self.recognizer.recognize(mp_image)
        #annotated_image = display_batch_of_images_with_gestures_and_hand_landmarks(mp_image, res)


        annotated_image = image.copy()
        for handedness, hand_landmarks, gesture in zip(res.handedness, res.hand_landmarks, res.gestures):
            if handedness[0].display_name == 'Left':
                print('Left category: ', gesture[0].category_name)
            elif handedness[0].display_name == 'Right':
                print('Right category: ', gesture[0].category_name)
            else:
                pass



            for hand_landmarks in res.hand_landmarks:
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
                        

        return annotated_image, annotated_image, annotated_image





class HandTracker(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        package_name = 'hand_tracker'
        package_path = get_package_share_directory(package_name)
        folder_path = os.path.join(package_path, 'task')
        gesture_path = os.path.join(folder_path, 'gesture_recognizer.task')
        tracker_path = os.path.join(folder_path, 'hand_landmarker.task')


        CAMERA_TOPIC = '/oak/rgb/image_raw'
        CAMERA_TOPIC = '/camera1/image_raw'
        CAMERA_TOPIC = '/image_raw'

        self.detector = Detector(tracker_path, gesture_path)

        self.publisher_ = self.create_publisher(HandTrack, 'hand_tracking', 10)
        self.publisher_1 = self.create_publisher(Image, 'image_results', 10)
        self.publisher_2 = self.create_publisher(Image, 'left_hand', 10)
        self.publisher_3 = self.create_publisher(Image, 'right_hand', 10)

        self.subscibrer_ = self.create_subscription(Image, CAMERA_TOPIC, self.listener_callback, 10)
        self.current_image = None

        self.cv_bridge = CvBridge()


        timer_frequency = 30 # Hz
        timer_period = 1.0/timer_frequency  # seconds

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg:Image):
        self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #self.current_image = cv2.cvtColor(self.current_image, cv2.COLOR_BGR2RGB)


    def timer_callback(self):
        
        if not isinstance(self.current_image, type(None)):

            print(self.current_image.shape)
            an, left, right = self.detector.single_detect(self.current_image.copy())
            

            an_msg = self.cv_bridge.cv2_to_imgmsg(an, encoding="rgb8")
            self.publisher_1.publish(an_msg)

            left_msgs = self.cv_bridge.cv2_to_imgmsg(left, encoding="rgb8")
            self.publisher_2.publish(left_msgs)
            
            right_msgs = self.cv_bridge.cv2_to_imgmsg(right, encoding="rgb8")
            self.publisher_3.publish(right_msgs)

            print('pl')

        self.current_image = None

        #self.get_logger().info('Publishing: "%s"' % msg.data)




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