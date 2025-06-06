from depthai_sdk import OakCamera
import depthai
from depthai_sdk.classes import DetectionPacket
from depthai_sdk.visualize.visualizer_helper import FramePosition, VisualizerHelper
import cv2

def callback(packet: DetectionPacket):
    #VisualizerHelper.print(packet.frame, 'BottomRight!', FramePosition.BottomRight)
    for d in packet.img_detections.detections:
        print("label: ", d.label)
        print("center: ", (d.xmax - d.xmin) / 2 + d.xmin, (d.ymax - d.ymin) / 2 + d.ymin)

with OakCamera() as oak:
    color = oak.create_camera('color')
    model_config = {
        'source': 'roboflow', # Specify that we are downloading the model from Roboflow
        'model':'detect-faces-m1pbd/6',
        'key':'3omR0u9ATOe8U1rQmZwo'
    }
    nn = oak.create_nn(model_config, color)
    oak.visualize([nn], fps=True, callback=callback)
    oak.start(blocking=True)

