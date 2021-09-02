#!/usr/bin/env python3

# Libraries
import rclpy
from rclpy.node import Node
import cv2
from pymba import *
from sensor_msgs.msg import Image
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge
import threading
import sys

from std_msgs.msg import Header
import tf2_ros
import geometry_msgs
from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class AVNode(Node):
    def __init__(self):
        super().__init__("av_camera_node")
        self.get_logger().info("AV Camera node is awake...")
        
        # Parameters declaration
        self.declare_parameter("cam_id", 1)

        # Class attributes
        self.cam_id = self.get_parameter("cam_id").value
        self.bridge = CvBridge()
        self.frame = []
        self.start_acquisition = True

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        # Publishers
        self.frame_pub = self.create_publisher(Image, "/parking_camera/raw_frame", 1)

        # Service: stop acquisition
        self.stop_service = self.create_service(CameraState, "/parking_camera/get_cam_state", self.acquisition_service)


    # This function stops/enable the acquisition stream
    def acquisition_service(self, request, response):
        self.start_acquisition = request.command_state
        response.cam_state = self.start_acquisition
        return response


    # This function save the current frame in a class attribute
    def get_frame(self):

        with Vimba() as vimba:

            cam_found = True

            try:
                # Open the cam and set the mode
                self.cam_obj = vimba.camera(0)
                self.cam_obj.open()

                # read a feature value
                feature = self.cam_obj.feature('ExposureAuto')
                feature.value = 'Continuous'

                '''
                # get feature value via feature object
                for feature_name in self.cam_obj.feature_names():
                    feature = self.cam_obj.feature(feature_name)

                    try:
                        value = feature.value
                        range_ = feature.range

                    except VimbaException as e:
                        value = e
                        range_ = None

                    print('\n\t'.join(
                        str(x) for x in (
                            feature_name,
                            'value: {}'.format(value),
                            'range: {}'.format(range_))
                        if x is not None))
                '''

            except:
                self.get_logger().info("No AlliedVision Alvium cameras found, check connection.")
                cam_found = False

            if cam_found:
                try:
                    self.cam_obj.arm("SingleFrame")
                    self.get_logger().info("Frame acquisition has started.")
                    
                    while self.start_acquisition:
                        current_frame = self.cam_obj.acquire_frame()
                        self.frame = current_frame.buffer_data_numpy()
                        self.publish_frame()

                    self.cam_obj.disarm()
                    self.cam_obj.close()
                except:
                    self.cam_obj.disarm()
                    self.cam_obj.close()

                
    # This function stops/enable the acquisition stream
    def exit(self):
        self.start_acquisition = False
        self.thread1.join()



    # Publisher function
    def publish_frame(self):
        
        if len(self.frame) == 0:
            return

        self.image_message = self.bridge.cv2_to_imgmsg(self.frame, encoding="mono8")
        self.image_message.header = Header()
        self.image_message.header.stamp = self.get_clock().now().to_msg()
        self.image_message.header.frame_id = "parking_camera_link"
        self.frame_pub.publish(self.image_message)



# Main loop function
def main(args=None):

    rclpy.init(args=args)
    node = AVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('AlliedVision Node stopped cleanly')
        node.exit()
    except BaseException:
        print('Exception in AlliedVision Node:', file=sys.stderr)
        raise
    finally:
        # Destroy the node explicitly
        # (optional - Done automatically when node is garbage collected)
        node.destroy_node()
        rclpy.shutdown() 


# Main
if __name__ == '__main__':
    main()
