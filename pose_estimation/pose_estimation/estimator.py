# HUA
# Contact: kfoteinos@hua.gr

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo, NavSatFix
from nav_msgs.msg import Odometry
from message_filters import Subscriber, ApproximateTimeSynchronizer
import json
import math
import numpy as np
from ultralytics import YOLO

EARTH_RADIUS = 6378137.0 # in meters

POSE_ESTIMATOR = "yolo26n-pose.pt"
FOCAL_LENGTH = 1 # typical values of focal length are <10 mm for ordinary cameras; therefore, the impact on the final results is negligible
NAV_TOPIC = "/fix"
ODOM_TOPIC = "/dog_odom"
DEPTH_TOPIC = "/camera_front/depth"
RGB_TOPIC = "/camera_front/color"
CAMERA_INFO = "/camera_front/camera_info"
OUTPUT_TOPIC = "/human_pose"

KEYPOINTS = [
    "Nose",
    "Left Eye",
    "Right Eye",
    "Left Ear",
    "Right Ear",
    "Left Shoulder",
    "Right Shoulder",
    "Left Elbow",
    "Right Elbow",
    "Left Wrist",
    "Right Wrist",
    "Left Hip",
    "Right Hip",
    "Left Knee",
    "Right Knee",
    "Left Ankle",
    "Right Ankle"
]


class Human_Pose_Estimator(Node):

    def __init__(self, pose_estimator:str=POSE_ESTIMATOR, color_topic:str=RGB_TOPIC, depth_topic:str=DEPTH_TOPIC, camera_info:str=CAMERA_INFO, nav_fix_topic:str=NAV_TOPIC, odom_topic:str=ODOM_TOPIC, output_topic:str=OUTPUT_TOPIC):
        super().__init__("human_pose_estimator")
        ApproximateTimeSynchronizer(
            fs=[
                Subscriber(self, Image, color_topic), 
                Subscriber(self, Image, depth_topic), 
                Subscriber(self, CameraInfo, camera_info),
                Subscriber(self, NavSatFix, nav_fix_topic), 
                Subscriber(self, Odometry, odom_topic)
            ],
            queue_size=10,
            slop=1e-2
        ).registerCallback(self.__main_callback)
        self.__publisher=self.create_publisher(
            msg_type = String,
            topic = output_topic,
            qos_profile = 10
        )
        self.__pose_estimator = YOLO(pose_estimator)
        self.__init_latitude = None
        self.__init_longitude = None


    def __detect_keypoints(self, image, depth_map, names = KEYPOINTS) -> list[dict]: # H x W x 3 nd arrays
        '''
        Parameters:
            image:      numpy array with dimensions height x width x 3 (H x W x 3)
            depth_map:  numpy array with dimensions height x width x 1 (H x W x 1)
        Returns:
            [
                {
                    Keys:       Keypoint name
                    Values:     [u coordinate (pixels), v coordinate (pixels), confidence ([0,1]), depth (units similar to the depth_map)] (primitives, not np arrays)
                },
                for every detected person
                ...
            ]
        '''
        result = self.__pose_estimator(image, verbose=False)[0].keypoints.data
        keypoints = []
        for person in range(len(result)):
            keypoints.append(dict({}))
            for i in range(len(names)):
                uvcd:list = result[person][i][[0,1]].numpy().astype(int).tolist()
                uvcd.append(result[person][i][2].item())
                try:
                    uvcd.append(depth_map[uvcd[1],uvcd[0]].item())
                except IndexError:
                    uvcd.append(0)
                keypoints[-1][names[i]] = uvcd
        return keypoints
    

    def __aggregate(self, keypoints:dict) -> float:
        '''
        Parameters:
            keypoints:  Keypoints of a single person in the format of __detect_keypoints output
        Returns:
            avg:    average depth of the keypoints (in the given depth measurement units)
            u:      average u (in pixels)
            v:      average v (in pixels)
        '''
        avg = u = v = 0
        for keypoint_name in keypoints.keys():
            if keypoints[keypoint_name][3] == 0:
                continue
            avg += keypoints[keypoint_name][3]
            u += keypoints[keypoint_name][0]
            v += keypoints[keypoint_name][1]
        assert len(keypoints) == len(KEYPOINTS)
        avg /= len(keypoints)
        return avg, u, v


    def __estimate_relative_location(self, u, v, depth, intrinsics, f=FOCAL_LENGTH) -> np.ndarray:
        '''
        Backprojects a point to 3D space
        Parameters:
            u:          x' (horizontal) (in pixels)
            v:          y' (vertical) (in pixels)
            depth:      disantce from the optical center (in mm)
            intrinsics: camera intrinsics (in pixels)
            f:          focal length (in mm)
        Returns:
            the x,y,z position in 3D space (in mm)
        '''
        Z = depth - f
        K = intrinsics
        P_2D_h = np.asarray([u, v, 1]) # homogeneous coordinates
        P_3D = np.linalg.inv(K) @ (Z * P_2D_h)
        return P_3D


    def __estimate_absolute_location(self, det_distance, cam_lat, cam_lon, orientation_x, orientation_y):
        '''
        Parameters:
            det_distance:   Distance between the detection and the camera (assume that the detection is in the line of the orientation of the camera) (in mm)
            cam_lat:        Current global latitude of the camera/robot
            cam_lon:        Current global longitude of the camera/robot
            orientation_x:  W.r.t. to the xy coordinate system that satisfies xx' // parallels, yy' // meridians (approximately) (*)
            orientation_y:  The same with orientation_x
        Returns:
            longitude:      GPS (degrees)
            latitude:       GPS (degrees)
        '''
        cam_x, cam_y = self.__global_to_xy_position(lat=cam_lat, lon=cam_lon)
        norm = math.sqrt(orientation_x ** 2 + orientation_y ** 2) * 1000 # in mm
        det_x = cam_x + det_distance * orientation_x / norm
        det_y = cam_y + det_distance * orientation_y / norm
        return self.__xy_to_global_position(x=det_x, y=det_y)


    def __xy_to_global_position(self, x, y) -> list:
        '''
        Parameters:
            x,y:        With origin the initial robot position and "orientation" the same with the "flatten" meridians/parallels (in mm)
        Returns:
            longitude:  GPS (degrees)
            latitude:   GPS (degrees)
        '''
        lat = self.__init_latitude + (y*1000 / EARTH_RADIUS) * (180.0 / math.pi)
        lon = self.__init_longitude + (x*1000 / (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))) * (180.0 / math.pi)
        return lon, lat
    

    def __global_to_xy_position(self, lat, lon):
        '''
        The inverse of the previous
        '''
        y = (lat - self.__init_latitude) * (math.pi / 180.0) * EARTH_RADIUS
        x = (lon - self.__init_longitude) * (math.pi / 180.0) * (EARTH_RADIUS * math.cos(math.radians(self.__init_latitude)))
        return x/1000, y/1000 # (in mm)


    def __register_initial_global_position(self, position:NavSatFix):
        if self.__init_latitude is None or self.__init_longitude is None:
            self.__init_latitude = position.latitude
            self.__init_longitude = position.longitude
            self.get_logger().info(f"Initial position in (latitude, longitude) = ({self.__init_latitude}, {self.__init_longitude})")


    def __quaternion_to_rpy(self, qx, qy, qz, qw):
        return {
            "roll": math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz),
            "pitch":math.asin(-2.0*(qx*qz - qw*qy)),
            "yaw":  math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        }


    def __main_callback(self, color_image:Image, depth_map:Image, intrinsics:CameraInfo, global_position:NavSatFix, odometry:Odometry):
        '''
        Parameters:
            color_image:    8-bit RGB image (H x W x 3)
            depth_map:      16UC1 in mm depth map (H x W x 2) of the same dimensions and aligned to the color_image
            intrinsics:     Camera intrinsics (the code uses only K matrix)
            global_position:Longitude/latitude (degrees)
            odomometry:     The code needs the "absolute" orientation (w.r.t. to the "standard" xy plane), as described in (*)
        Publishes:
            See README
        '''
        
        assert color_image.height == depth_map.height and color_image.width == depth_map.width
        
        self.__register_initial_global_position(global_position)
        self.get_logger().info(f"Received RGBD frames of size {color_image.height} x {color_image.width} (H x W) at {self.__global_to_xy_position(lat=global_position.latitude, lon=global_position.longitude)} (mm)")
        
        color_image_array = np.asarray(color_image.data, dtype=np.float32).reshape((color_image.height, color_image.width, 3)) # H x W x 3
        depth_map_array = np.asarray(np.frombuffer(depth_map.data,dtype=np.uint16), dtype=np.float32).reshape((depth_map.height, depth_map.width, 1)) # H x W x 1

        all_keypoints = self.__detect_keypoints(image=color_image_array, depth_map=depth_map_array)
        features = []
        for single_person_keypoints in all_keypoints:
            depth, u, v = self.__aggregate(single_person_keypoints)
            relative_position = self.__estimate_relative_location(u=u,v=v,depth=depth,intrinsics=np.asarray(intrinsics.k).reshape((3,3)))
            angle = math.radians(self.__quaternion_to_rpy(odometry.pose.pose.orientation.x,odometry.pose.pose.orientation.y,odometry.pose.pose.orientation.z,odometry.pose.pose.orientation.w)["yaw"])
            det_global_position = self.__estimate_absolute_location(depth, global_position.latitude, global_position.longitude, math.cos(angle), math.sin(angle))
            features.append({
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": det_global_position
                },
                "properties": {
                    "depth":depth,
                    "timestamp":self.get_clock().now().nanoseconds,
                    "keypoints_and_depths": single_person_keypoints,
                    "relative_position": relative_position.tolist()
                }
            })
        
        self.__publisher.publish(String(data=json.dumps({
            "type": "FeatureCollection",
            "features":features
        })))


def main():
    try:
        rclpy.init()
        rclpy.spin(node=Human_Pose_Estimator())
    except (ExternalShutdownException, KeyboardInterrupt):
        pass


if __name__ == '__main__':
    main()
