import depthai as dai
import numpy as np
import cv2
import socket
import struct

# PC IP and port
PC_IP = "192.168.185.100"
PORT = 9999

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((PC_IP, PORT))

pipeline = dai.Pipeline()

# Nodes
camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)

# Stream names
xoutRgb.setStreamName("rgb")
xoutDepth.setStreamName("depth")

# Camera setup
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setPreviewSize(416, 416)
camRgb.setInterleaved(False)
camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setCamera("left")
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setCamera("right")

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
stereo.setSubpixel(True)

# Linking
camRgb.preview.link(xoutRgb.input)
monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)

with dai.Device(pipeline) as device:
    rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    while True:
        try:
            rgbFrame = rgbQueue.get().getCvFrame()
            depthFrame = depthQueue.get().getFrame().astype(np.uint16)

            # Encode RGB image
            _, rgb_encoded = cv2.imencode('.jpg', rgbFrame)
            rgb_bytes = rgb_encoded.tobytes()

            # Send RGB with tag
            client_socket.send(b"RGB_")
            client_socket.send(struct.pack(">I", len(rgb_bytes)))
            client_socket.send(rgb_bytes)

            # Send raw depth with tag
            h, w = depthFrame.shape
            client_socket.send(b"DEPT")
            client_socket.send(struct.pack(">II", h, w))
            client_socket.send(depthFrame.tobytes())

        except BrokenPipeError:
            print("PC disconnected.")
            break
        except Exception as e:
            print("Streaming error:", e)
            break

client_socket.close()
