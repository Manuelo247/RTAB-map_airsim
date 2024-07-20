#!/usr/bin/env python3

import airsim
import rospy
import configparser
import os

import airsim

import numpy as np
import rospy

from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, CameraInfo  # Asegúrate de importar CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, TransformStamped
import tf2_ros, tf

from cv_bridge import CvBridge
import os
import signal
import sys


def readConfig():
    # Leer IP y puerto del archivo de configuración
    config = configparser.ConfigParser()
    config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')
    
    ip = config['airsim']['ip']
    ip = os.path.expandvars(ip)  # Expandir variables de entorno en IP
    port = int(config['airsim']['port'])
    
    return ip, port

def connectClient():
    global client  # Variable global para el cliente de AirSim
    ip, port = readConfig()  # Obtener IP y puerto
    
    client = airsim.MultirotorClient(ip=ip, port=port)  # Conectar con AirSim
    # client.confirmConnection()

def connectAPI():
    connectClient()  # Conectar al simulador
    
    client.enableApiControl(True)  # Habilitar control de la API
    client.armDisarm(True)  # Armar el dron
    
    return client

def disconnectAPI():
    client.armDisarm(False)  # Desarmar el dron
    client.enableApiControl(False)  # Deshabilitar control de la API


client = connectAPI()

if not client:
    rospy.logerr("Failed to connect to AirSim API")  # Error al conectar con AirSim
    print("Failed to connect to AirSim API")

import rospy
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header

def get_cameraInfo(size):
    camera_info = client.simGetCameraInfo("0")
    current_time = rospy.Time.now()

    cam_info_msg = CameraInfo()
    cam_info_msg.header.seq = 0  # Placeholder
    cam_info_msg.header.stamp = current_time  # Usar el tiempo actual
    cam_info_msg.header.frame_id = 'camera_link'
    cam_info_msg.width = size[0]  # Ajusta según sea necesario
    cam_info_msg.height = size[1]  # Ajusta según sea necesario

    if camera_info.fov <= 0:
        rospy.logwarn("FOV is non-positive or zero. Adjust the camera settings.")
        fx = size[0] / 2.0
        fy = size[1] / 2.0
    else:
        fx = size[0] / (2 * np.tan(camera_info.fov / 2.0))
        fy = size[1] / (2 * np.tan(camera_info.fov / 2.0))
        
    cx = size[0] / 2
    cy = size[1] / 2
    
    cam_info_msg.K = [
        fx, 0, cx,
        0, fy, cy,
        0, 0, 1
    ]
    cam_info_msg.P = [
        fx, 0, cx, 0,
        0, fy, cy, 0,
        0, 0, 1, 0
    ]
    
    cam_info_msg.distortion_model = 'plumb_bob'  # Modelo típico de distorsión
    cam_info_msg.D = [0, 0, 0, 0, 0]  # Parámetros de distorsión (k1, k2, t1, t2, k3)
    cam_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Matriz de rotación (identidad aquí)
    cam_info_msg.binning_x = 0
    cam_info_msg.binning_y = 0
    cam_info_msg.roi.x_offset = 0
    cam_info_msg.roi.y_offset = 0
    cam_info_msg.roi.height = 0
    cam_info_msg.roi.width = 0
    cam_info_msg.roi.do_rectify = False
    
    transform = TransformStamped()
    transform.header.stamp = current_time
    transform.header.frame_id = "odom"  # O el marco de referencia deseado
    transform.child_frame_id = "camera_link"

    if (camera_info.pose is None or 
        camera_info.pose.position is None or 
        camera_info.pose.orientation is None):
        rospy.logwarn("Pose information is incomplete or missing.")
        transform.transform.translation.x = 0
        transform.transform.translation.y = 0
        transform.transform.translation.z = 0
        transform.transform.rotation.x = 0
        transform.transform.rotation.y = 0
        transform.transform.rotation.z = 0
        transform.transform.rotation.w = 1
    else:
        transform.transform.translation.x = camera_info.pose.position.x_val
        transform.transform.translation.y = camera_info.pose.position.y_val
        transform.transform.translation.z = -camera_info.pose.position.z_val

        transform.transform.rotation.x = camera_info.pose.orientation.x_val
        transform.transform.rotation.y = camera_info.pose.orientation.y_val
        transform.transform.rotation.z = camera_info.pose.orientation.z_val
        transform.transform.rotation.w = camera_info.pose.orientation.w_val
    
    # tf_broadcaster.sendTransform(
    #     (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
    #     (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
    #     current_time,
    #     transform.child_frame_id,
    #     transform.header.frame_id
    # )

    return cam_info_msg

def test_publish_camera_info():
    rospy.init_node('camera_info_test_publisher', anonymous=True)
    info_pub = rospy.Publisher('/airsim/camera_info', CameraInfo, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        cam_info_msg = get_cameraInfo([100, 100])

        info_pub.publish(cam_info_msg)
        rospy.loginfo("Published camera info")
        rate.sleep()

if __name__ == '__main__':
    try:
        test_publish_camera_info()
    except rospy.ROSInterruptException:
        pass
    
