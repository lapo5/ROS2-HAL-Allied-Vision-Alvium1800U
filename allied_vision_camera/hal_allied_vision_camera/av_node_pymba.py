#!/usr/bin/env python3

# Libraries
import threading
import sys
import numpy as np
import cv2 as cv
from pymba import *

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header

from cv_bridge import CvBridge

from allied_vision_camera_interfaces.srv import CameraState

# Class definition of the calibration function
class AVNode(Node):
    def __init__(self):
        super().__init__("av_node")
        
        self.declare_parameter("camera_name", "auto")

        self.camera_name = self.get_parameter("camera_name").value
        self.bridge = CvBridge()
        self.frame = []
        self.start_acquisition = True

        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        self.declare_parameter("publishers.raw_frame", "/camera/raw_frame")
        self.raw_frame_topic = self.get_parameter("publishers.raw_frame").value

        self.declare_parameter("services.stop_camera", "/camera/stop_camera")
        self.stop_cam_service = self.get_parameter("services.stop_camera").value

        self.declare_parameter("rotation_angle", "0.0")
        self.rotation_angle = float(self.get_parameter("rotation_angle").value)

        self.declare_parameter("frames.camera_link", "camera_link")
        self.camera_link = self.get_parameter("frames.camera_link").value

        # Publishers
        self.frame_pub = self.create_publisher(Image, self.raw_frame_topic, 1)

        # Service: stop acquisition
        self.stop_service = self.create_service(CameraState, self.stop_cam_service, self.acquisition_service)
        self.get_logger().info("[AV Camera] Node Ready")


    def acquisition_service(self, request, response):
        self.start_acquisition = request.command_state
        response.cam_state = self.start_acquisition
        return response


    # This function save the current frame in a class attribute
    def get_frame(self):

        with Vimba() as vimba:

            cam_found = True

            try:
                self.get_logger().info("Available Cameras: {0}".format(vimba.camera_ids()))
                
                # Open the cam and set the mode
                if self.camera_name == "auto":
                    self.get_logger().info("Opening First Camera Available")
                    self.cam_obj = vimba.camera(0)
                else:

                    cam_id = 0
                    cam_count = len(vimba.camera_ids())
                    for cam_name_id in vimba.camera_ids():
                        if cam_name_id == self.camera_name:
                            self.cam_obj = vimba.camera(cam_id)
                            break
                        cam_id = cam_id + 1

                    if cam_id >= cam_count:
                        self.get_logger().info("Camera with name {0} not found".format(self.camera_name))

                self.cam_obj.open()


                for feature_name in self.cam_obj.feature_names():
                    feature = self.cam_obj.feature(feature_name)
                    #print(feature.info)

                # read a feature value
                feature = self.cam_obj.feature('ExposureAuto')
                feature.value = 'Continuous'

            except:
                self.get_logger().info("[AV Camera] No AlliedVision Alvium cameras found, check connection.")
                cam_found = False

            if cam_found:
                try:
                    self.cam_obj.arm("SingleFrame")
                    self.get_logger().info("[AV Camera] Frame acquisition has started.")
                    
                    while self.start_acquisition:
                        current_frame = self.cam_obj.acquire_frame()
                        self.frame = current_frame.buffer_data_numpy()
                        self.publish_frame()

                    self.cam_obj.disarm()
                    self.cam_obj.close()
                except:
                    self.cam_obj.disarm()
                    self.cam_obj.close()


    def exit(self):
        self.start_acquisition = False
        self.thread1.join()


    def publish_frame(self):
        
        if len(self.frame) == 0:
            self.get_logger().info("[AV Camera] No Image Returned")
            return
        
        if self.rotation_angle != 0.0:
            image_center = tuple(np.array(self.frame.shape[1::-1]) / 2)
            rot_mat = cv.getRotationMatrix2D(image_center, self.rotation_angle, 1.0)
            self.frame = cv.warpAffine(self.frame, rot_mat, self.frame.shape[1::-1], flags=cv.INTER_LINEAR)

        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
        self.image_message.header = Header()
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_message.header.frame_id = self.camera_link
        self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = AVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('[AV Camera] Node stopped cleanly')
        node.exit()
    except BaseException:
        node.get_logger().info('[AV Camera] Exception:', file=sys.stderr)
        node.exit()
        raise
    finally:
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()
