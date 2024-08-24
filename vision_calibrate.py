#!/usr/bin/env python3
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at https://mozilla.org/MPL/2.0/.

import cv2
import depthai as dai
import numpy as np
import time

hmin = 15
hmax = 175
smin = 175
smax = 255
vmin = 175
vmax = 255

def on_hmin(val):
    globals()['hmin'] = val

def on_hmax(val):
    globals()['hmax'] = val

def on_smin(val):
    globals()['smin'] = val

def on_smax(val):
    globals()['smax'] = val

def on_vmin(val):
    globals()['vmin'] = val

def on_vmax(val):
    globals()['vmax'] = val

# Create pipeline
pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
camRgb.setIspScale(1, 3)
camRgb.setVideoSize(1280, 720)

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)

# Connect to device and start pipeline
with dai.Device(pipeline, usb2Mode=True) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    cv2.namedWindow("Range")
    cv2.resizeWindow("Range", 640, 240)

    cv2.createTrackbar("H min", "Range", hmin, 180, on_hmin)
    cv2.createTrackbar("H max", "Range", hmax, 180, on_hmax)
    cv2.createTrackbar("S min", "Range", smin, 255, on_smin)
    cv2.createTrackbar("S max", "Range", smax, 255, on_smax)
    cv2.createTrackbar("V min", "Range", vmin, 255, on_vmin)
    cv2.createTrackbar("V max", "Range", vmax, 255, on_vmax)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, [7, 7])
    while True:

        lower1 = np.array([0, smin, vmin])
        upper1 = np.array([hmin, smax, vmax])
        lower2 = np.array([hmax, smin, vmin])
        upper2 = np.array([180, smax, vmax])

        frame = video.get().getCvFrame()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1_raw = cv2.inRange(hsv, lower1, upper1)
        mask2_raw = cv2.inRange(hsv, lower2, upper2)
        mask_raw = cv2.bitwise_or(mask1_raw, mask2_raw)
        mask = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, kernel)

        #maxes = np.full(hsv.shape[:2], 255, dtype='uint8')

        #(H, S, V) = cv2.split(hsv)
        res = cv2.bitwise_and(frame, frame, mask=mask)
        #res2 = cv2.merge([res, maxes, maxes])

        #cv2.imshow("H", cv2.cvtColor(res2, cv2.COLOR_HSV2BGR))
        cv2.imshow("masked", res)

        # Get BGR frame from NV12 encoded video frame to show with opencv
        # Visualizing the frame on slower hosts might have overhead
        #cv2.imshow("video", videoIn.getCvFrame())
        print(f'{time.time()} got frame')

        if cv2.waitKey(1) == ord('q'):
            break
