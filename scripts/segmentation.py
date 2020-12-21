#!/usr/bin/python3.6
import sys
from openvino.inference_engine import IECore
import logging as log
import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ROSInterruptException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

classes_color_map = [
    (150, 150, 150),
    (58, 55, 169),
    (211, 51, 17),
    (157, 80, 44),
    (23, 95, 189),
    (210, 133, 34),
    (76, 226, 202),
    (101, 138, 127),
    (223, 91, 182),
    (80, 128, 113),
    (235, 155, 55),
    (44, 151, 243),
    (159, 80, 170),
    (239, 208, 44),
    (128, 50, 51),
    (82, 141, 193),
    (9, 107, 10),
    (223, 90, 142),
    (50, 248, 83),
    (178, 101, 130),
    (71, 30, 204)
]

class Segmentation(Node):
    def __init__(self):
        rclpy.init(args = None)
        super().__init__("segmentation_node")

        self.setup_inference_engine()

        self.pub = self.create_publisher(Image, "segmentation_image", 1)
        self.sub = self.create_subscription(Image, "/camera/image_raw", self.inference_cb, 1)
        self.hz = 0.3

        self.bridge = CvBridge()

    def setup_inference_engine(self):
        # Createing Inference Engine
        self.get_logger().info("Creating Inference Engine")
        ie = IECore()

        # Loading network
        self.get_logger().info("Loading network")
        model="/home/openvino/share/ml/fcn8s/fcn8s-heavy-pascal"
        self.net = ie.read_network(model=model+".xml", weights=model+".bin")

        # Preparing input blobs
        self.get_logger().info("Preparing input blobs")
        self.input_blob = next(iter(self.net.inputs))
        self.out_blob = next(iter(self.net.outputs))
        #net.batch_size = len(args.input)

        # NB: This is required to load the image as uint8 np.array
        #     Without this step the input blob is loaded in FP32 precision,
        #     this requires additional operation and more memory.
        self.net.inputs[self.input_blob].precision = "U8"

        # Read and pre-process input images
        self.model_n, self.model_c, self.model_h, self.model_w = self.net.inputs[self.input_blob].shape

        # Loading model to plugin
        self.get_logger().info("Loading model to the plugin")
        self.exec_net = ie.load_network(network=self.net, device_name='GPU', num_requests=1)
        self.get_logger().info("Finish loading model")

    def inference_cb(self, msg):
        print("start inference")
        try:
            start_time = time.time()
            image_origin = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert image from cv2 to openVINO
            height, width = image_origin.shape[:2]
            image = cv2.resize(image_origin, (self.model_w, self.model_h))
            image = image.transpose((2, 0, 1))  # Change data layout from HWC to CHW
            image = image.reshape((self.model_n, self.model_c, self.model_h, self.model_w))

            # Start inference
            self.get_logger().info("Starting inference")
            res = self.exec_net.infer(inputs={self.input_blob: image})

            # Processing output blob
            res = res[self.out_blob]
            if len(res.shape) == 3:
                res = np.expand_dims(res, axis=1)
            if len(res.shape) == 4:
                _, _, out_h, out_w = res.shape
            else:
                raise Exception("Unexpected output blob shape {}. Only 4D and 3D output blobs are supported".format(res.shape))


            for batch, data in enumerate(res):
                classes_map = np.zeros(shape=(out_h, out_w, 3), dtype=np.int)
                for i in range(out_h):
                    for j in range(out_w):
                        pixel_class = np.argmax(data[:, i, j])
                        classes_map[i, j, :] = classes_color_map[min(pixel_class, 20)]


            classes_map = classes_map.astype(np.uint8)
            classes_map = cv2.resize(classes_map, (width, height))
            image_msg = self.bridge.cv2_to_imgmsg(classes_map, "bgr8")
            image_msg.header.frame_id = "camera"
            self.pub.publish(image_msg)

            infer_time = time.time() - start_time
            print("infer time is ", infer_time)

            sleep_time = 1/self.hz - infer_time
            if(sleep_time > 0):
                time.sleep(sleep_time)

        except Exception as e:
            print(e)

def main(args = None):
    segmentation = Segmentation()

    rclpy.spin(segmentation)

    segmentation.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    try:
        main()
    except ROSInterruptException: 
        pass

