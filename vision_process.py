#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

import json
import time
import select
import sys

# thresholds
hmin = 15
hmax = 175
smin = 160
smax = 255
vmin = 160
vmax = 255

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

# debug mode?

debug = False
if len(sys.argv) > 1 and sys.argv[1] == '--debug':
    debug = True

# Connect to device and start pipeline
with dai.Device(pipeline, usb2Mode=False) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, [5, 5])

    lower1 = np.array([0, smin, vmin])
    upper1 = np.array([hmin, smax, vmax])
    lower2 = np.array([hmax, smin, vmin])
    upper2 = np.array([180, smax, vmax])

    while True:
        videoIn = video.get()
        frame = videoIn.getCvFrame()

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1_raw = cv2.inRange(hsv, lower1, upper1)
        mask2_raw = cv2.inRange(hsv, lower2, upper2)
        mask_raw = cv2.bitwise_or(mask1_raw, mask2_raw)
        mask = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, kernel)

        #frame_masked = cv2.bitwise_and(frame, frame, mask=mask)
        
        candidate = { 'found' : False }
        frame_time = time.time()

        n, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
        for i in range(1, n):
            x = stats[i, cv2.CC_STAT_LEFT]
            y = stats[i, cv2.CC_STAT_TOP]
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            a = stats[i, cv2.CC_STAT_AREA]
            (cX, cY) = centroids[i]
            
            # reject too small regions
            if a < 100:
                continue

            # track the largest blob
            if not candidate['found'] or candidate['area'] < a:
                candidate = { 'found' : True, 'time': frame_time, 'area' : int(a), 'x' : int(cX), 'y' : int(cY) }
            
            # for debugging, draw blob bounds into the frame
            if debug:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(frame, f'Area = {a}', (x, y + h + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        print(f'{json.dumps(candidate)}', flush=True)

        # display frame if in debug mode
        if debug:
            cv2.imshow("mask", mask)
            cv2.imshow("frame", frame)
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            # exit on data input
            if select.select([sys.stdin, ], [], [], 0.0)[0]:
                break
