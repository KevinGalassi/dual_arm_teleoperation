#!/usr/bin/env python3

import cv2
import depthai as dai

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
