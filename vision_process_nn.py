#!/usr/bin/env python3
"""
The code is edited from docs (https://docs.luxonis.com/projects/api/en/latest/samples/Yolo/tiny_yolo/)
We add parsing from JSON files that contain configuration
"""

from pathlib import Path
import sys
import cv2
import depthai as dai
import numpy as np
import time
import argparse
import json
import blobconverter
import datetime
import math

# parse config
configPath = Path('coneslayer/coneslayer.json')
if not configPath.exists():
    raise ValueError("Path {} does not exist!".format(configPath))

with configPath.open() as f:
    config = json.load(f)
nnConfig = config.get("nn_config", {})

# parse input shape
if "input_size" in nnConfig:
    W, H = tuple(map(int, nnConfig.get("input_size").split('x')))

# extract metadata
metadata = nnConfig.get("NN_specific_metadata", {})
classes = metadata.get("classes", {})
coordinates = metadata.get("coordinates", {})
anchors = metadata.get("anchors", {})
anchorMasks = metadata.get("anchor_masks", {})
iouThreshold = metadata.get("iou_threshold", {})
confidenceThreshold = metadata.get("confidence_threshold", {})

# parse labels
nnMappings = config.get("mappings", {})
labels = nnMappings.get("labels", {})

# get model path
nnPath = 'coneslayer/coneslayer_openvino_2022.1_6shave.blob'
if not Path(nnPath).exists():
    raise Exception()

# sync outputs
syncNN = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
camRgbEnc = pipeline.create(dai.node.VideoEncoder)
detectionNetwork = pipeline.create(dai.node.YoloDetectionNetwork)
#xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgbEnc = pipeline.create(dai.node.XLinkOut)
nnOut = pipeline.create(dai.node.XLinkOut)

#xoutRgb.setStreamName("rgb")
xoutRgbEnc.setStreamName("h265")
nnOut.setStreamName("nn")

# Properties
camRgb.setPreviewSize(W, H)
camRgb.setPreviewKeepAspectRatio(True)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
camRgb.setFps(30)

# Video encoder specific settings
camRgbEnc.setDefaultProfilePreset(camRgb.getFps(), dai.VideoEncoderProperties.Profile.H265_MAIN)

# Network specific settings
detectionNetwork.setConfidenceThreshold(confidenceThreshold)
detectionNetwork.setNumClasses(classes)
detectionNetwork.setCoordinateSize(coordinates)
detectionNetwork.setAnchors(anchors)
detectionNetwork.setAnchorMasks(anchorMasks)
detectionNetwork.setIouThreshold(iouThreshold)
detectionNetwork.setBlobPath(nnPath)
detectionNetwork.setNumInferenceThreads(2)
detectionNetwork.input.setBlocking(False)

# Linking
camRgb.preview.link(detectionNetwork.input)
camRgb.video.link(camRgbEnc.input)
#detectionNetwork.passthrough.link(xoutRgb.input)
detectionNetwork.out.link(nnOut.input)
camRgbEnc.bitstream.link(xoutRgbEnc.input)

# Field of view helpers
def getHFov(intrinsics, width):
    fx = intrinsics[0][0]
    fov = 2 * 180 / (math.pi) * math.atan(width * 0.5 / fx)
    return fov

def getVFov(intrinsics, height):
    fy = intrinsics[1][1]
    fov = 2 * 180 / (math.pi) * math.atan(height * 0.5 / fy)
    return fov

# Connect to device and start pipeline
with dai.Device(pipeline, usb2Mode=True) as device:

    # Output queues will be used to get the rgb frames and nn data from the outputs defined above
    #qRgb = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    qDet = device.getOutputQueue(name="nn", maxSize=4, blocking=False)
    qH265 = device.getOutputQueue(name='h265', maxSize=4, blocking=False)

    videoName = datetime.datetime.now().strftime('/home/nlewis/Videos/capture_%Y-%m-%d_%H-%M-%S.h265')
    videoFile = open(videoName, "w")

    frame = None
    detections = []
    startTime = time.monotonic()
    counter = 0
    color2 = (255, 255, 255)

    # compute the field of view of the camera
    width = camRgb.getPreviewWidth()
    height = camRgb.getPreviewHeight()
    real_width = camRgb.getVideoWidth()
    real_height = camRgb.getVideoHeight()

    calib = device.readCalibration()
    intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.RGB, real_width, real_height)

    hfov = getHFov(intrinsics, width * (real_height/height))
    vfov = getVFov(intrinsics, height * (real_height/height))

    # nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
    def frameNorm(bbox):
        normVals = np.full(len(bbox), width)
        normVals[::2] = height
        return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

    #def displayFrame(name, frame, detections):
    #    color = (255, 0, 0)
    #    i = 0
    #    for detection in detections:
    #        i = i + 1
    #        bbox = frameNorm((detection.xmin, detection.ymin, detection.xmax, detection.ymax))
    #        print(f"detection {i}: {bbox}")
    #        cv2.putText(frame, labels[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #        cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
    #        cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
    #    # Show the frame
    #    cv2.imshow(name, frame)

    last_frame_time = time.time() - 1
    while True:
        queues = ('nn', 'h265')
        q = device.getQueueEvent(queues)

        # handle video queues
        if q == 'h265':
            while qH265.has():
                qH265.get().getData().tofile(videoFile)
            continue
        elif q == 'nn':
            pass
        else:
            continue

        # handle nn queue
        frame_time = time.time()
        fps = 1.0 / (frame_time - last_frame_time)
        last_frame_time = frame_time

        # print detections
        detections = qDet.get().detections
        candidate = { 'time' : frame_time, 'fps' : fps, 'found' : False, 'iw' : width, 'ih' : height }
        for detection in detections:
            bbox = frameNorm((detection.xmin, detection.ymin, detection.xmax, detection.ymax))

            # compute stuff
            x = bbox[0]
            y = bbox[1]
            w = (bbox[2] - x)
            h = (bbox[3] - y)
            a = w * h * detection.confidence**2

            cX = ((float(x + w/2) / width) - 0.5) * hfov
            cY = ((float(y + h/2) / height) - 0.5) * vfov

            # reject too small regions
            if a < 100:
                continue

            # track the largest blob
            if not candidate['found'] or candidate['area'] < a:
                candidate['found'] = True

                candidate['area'] = int(a)
                candidate['x'] = float(cX)
                candidate['y'] = float(cY)

                candidate['rect'] = {}
                candidate['rect']['x'] = int(x)
                candidate['rect']['y'] = int(y)
                candidate['rect']['w'] = int(w)
                candidate['rect']['h'] = int(h)

        print(f'{json.dumps(candidate)}', flush=True)
