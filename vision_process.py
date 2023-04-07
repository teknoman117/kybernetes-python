#!/usr/bin/env python3

import cv2
import depthai as dai
import numpy as np

import datetime
import json
import time
import select
import sys
import math

# thresholds
hmin = 15
hmax = 175
smin = 175
smax = 255
vmin = 175
vmax = 255

image_width = 1280
image_height = 720

# debug mode
debug = False
if len(sys.argv) > 1 and sys.argv[1] == '--debug':
    debug = True

# Create pipeline
pipeline = dai.Pipeline()

# Color camera
camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
camRgb.setIspScale(1, 3)
camRgb.setVideoSize(image_width, image_height)
camRgb.setFps(30)

xoutVideo = pipeline.create(dai.node.XLinkOut)
xoutVideo.setStreamName("video")
xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

camRgb.video.link(xoutVideo.input)

# Video Encoders if not in debug mode
if not debug:
    videoEnc = pipeline.create(dai.node.VideoEncoder)
    videoEnc.setDefaultProfilePreset(camRgb.getFps(), dai.VideoEncoderProperties.Profile.H265_MAIN)
    xoutVideoEnc = pipeline.create(dai.node.XLinkOut)
    xoutVideoEnc.setStreamName('h265')
    camRgb.video.link(videoEnc.input)
    videoEnc.bitstream.link(xoutVideoEnc.input)

# Connect to device and start pipeline
with dai.Device(pipeline, usb2Mode=True) as device:
    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)

    if not debug:
        bitstream = device.getOutputQueue(name='h265', maxSize=int(camRgb.getFps()), blocking=True)

    videoName = datetime.datetime.now().strftime('/home/nlewis/Videos/capture_%Y-%m-%d_%H-%M-%S.h265')
    videoFile = open(videoName, "w")

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, [5, 5])

    lower1 = np.array([0, smin, vmin])
    upper1 = np.array([hmin, smax, vmax])
    lower2 = np.array([hmax, smin, vmin])
    upper2 = np.array([180, smax, vmax])
    
    calib = device.readCalibration()
    intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, dai.Size2f(image_width, image_height))
    
    hfov =  2 * 180 / (math.pi) * math.atan(image_width * 0.5 / intrinsics[0][0])
    
    print(f'hfov = {hfov}')

    last_frame_time = time.time() - 1
    while True:
        # no video encoder in debug mode
        if not debug:
            queues = ("video", "h265")
        else:
            queues = ("video")
        queueName = device.getQueueEvent(queues)

        # action based on which queue has results
        if queueName == "h265":
            # write out video
            while bitstream.has():
                bitstream.get().getData().tofile(videoFile)
        elif queueName == "video":
            pass
        else:
            continue

        # look for cone
        frame = video.get().getCvFrame()
        frame_time = time.time()
        fps = 1.0 / (frame_time - last_frame_time)
        last_frame_time = frame_time

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1_raw = cv2.inRange(hsv, lower1, upper1)
        mask2_raw = cv2.inRange(hsv, lower2, upper2)
        mask_raw = cv2.bitwise_or(mask1_raw, mask2_raw)
        mask = cv2.morphologyEx(mask_raw, cv2.MORPH_OPEN, kernel)

        candidate = { 'time' : frame_time, 'fps' : fps, 'found' : False, 'iw' : image_width, 'ih' : image_height }
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
                candidate['found'] = True

                candidate['area'] = int(a)
                candidate['x'] = int(cX)
                candidate['y'] = int(cY)

                candidate['rect'] = {}
                candidate['rect']['x'] = int(x)
                candidate['rect']['y'] = int(y)
                candidate['rect']['w'] = int(w)
                candidate['rect']['h'] = int(h)

            # for debugging, draw blob bounds into the frame
            if debug:
                cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 3)
                cv2.putText(frame, f'Area = {a}', (x, y + h + 5), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        print(f'{json.dumps(candidate)}', flush=True)

        if debug:
            # display frame if in debug mode
            cv2.imshow("mask", mask)
            cv2.imshow("frame", frame)

            # exit on 'q' being pressed
            if cv2.waitKey(1) == ord('q'):
                break
        else:
            # exit on data input
            if select.select([sys.stdin, ], [], [], 0.0)[0]:
                break
