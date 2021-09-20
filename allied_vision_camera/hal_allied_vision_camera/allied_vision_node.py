#!/usr/bin/env python3

# Libraries
import sys
import threading
import cv2
import rclpy
from rclpy.node import Node
from vimba import *
from typing import Optional
from allied_vision_camera_interfaces.srv import CameraState
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Header

#import tf2_ros
#import geometry_msgs
#from scipy.spatial.transform import Rotation as R

# Class definition of the calibration function
class AVNode(Node):
    def __init__(self):
        super().__init__("av_camera_node")
        self.get_logger().info("AV Camera node is awake...")
        
        # Parameters declaration
        self.declare_parameter("cam_id", 0)

        # Class attributes
        self.cam_id = self.get_parameter("cam_id").value
        self.bridge = CvBridge()
        self.frame = []
        self.start_acquisition = True

        # Acquisition thread
        self.thread1 = threading.Thread(target=self.get_frame, daemon=True)
        self.thread1.start()

        # Publishers
        self.frame_pub = self.create_publisher(Image, "/arm_camera/raw_frame", 1)

        # Service: stop acquisition
        self.stop_service = self.create_service(CameraState, "/arm_camera/get_cam_state", self.acquisition_service)


    # This function stops/enable the acquisition stream
    def acquisition_service(self, request, response):
        self.start_acquisition = request.command_state
        response.cam_state = self.start_acquisition
        return response
    
    
    def get_camera(self, camera_id: Optional[str]) -> Camera:
        with Vimba.get_instance() as vimba:
            if camera_id:
                try:
                    return vimba.get_camera_by_id(camera_id)

                except VimbaCameraError:
                    self.get_logger().info("Failed to access Camera [" + camera_id + "]. Check connection.")
                    self.cam_found = False

            else:
                cams = vimba.get_all_cameras()
                if not cams:
                    self.get_logger().info("No Cameras accessible.")
                    self.cam_found = False

                return cams[0]

    

    def setup_camera(self):
        with self.cam_obj:
            # Enable auto exposure time setting if camera supports it
            try:
                self.cam_obj.ExposureAuto.set('Continuous')
            except (AttributeError, VimbaFeatureError):
                pass
            
            # Enable white balancing if camera supports it
            try:
                self.cam_obj.BalanceWhiteAuto.set('Continuous')
            except (AttributeError, VimbaFeatureError):
                pass

            '''
            # Query available, open_cv compatible pixel formats
            # prefer color formats over monochrome formats
            cv_fmts = intersect_pixel_formats(cam.get_pixel_formats(), OPENCV_PIXEL_FORMATS)
            color_fmts = intersect_pixel_formats(cv_fmts, COLOR_PIXEL_FORMATS)

            if color_fmts:
                cam.set_pixel_format(color_fmts[0])
            else:
                mono_fmts = intersect_pixel_formats(cv_fmts, MONO_PIXEL_FORMATS)

                if mono_fmts:
                    cam.set_pixel_format(mono_fmts[0])
                else:
                    abort('Camera does not support a OpenCV compatible format natively. Abort.')
            '''

    

    def get_frame(self):
    
        with Vimba.get_instance () as vimba :
        #    cams = vimba. get_all_cameras ()
            self.cam_obj = self.get_camera("")
            self.setup_camera()

            self.get_logger().info("Frame acquisition has started.")
            with self.cam_obj as cam: 
            
                while self.start_acquisition:
                    current_frame = cam.get_frame()
                    self.frame = current_frame.as_numpy_ndarray()
                    self.publish_frame()

                

    # This function save the current frame in a class attribute
    def get_frame_old(self):

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
        self.image_message.header.frame_id = "arm_camera_link"
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
