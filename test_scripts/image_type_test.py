#!/usr/bin/env python3

# Este codigo sirve para verificar que tipo de imagen 
# soporta una camara especifica regresada por airsim

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
import tf2_ros

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

def get_image_type_name(image_type):
    return {
        airsim.ImageType.Scene: "scene",
        airsim.ImageType.DepthPerspective: "depth_perspective",
        airsim.ImageType.DepthPlanar: "depth_planar",
        airsim.ImageType.DepthVis: "depth_vis",
        airsim.ImageType.Segmentation: "segmentation",
        airsim.ImageType.SurfaceNormals: "surface_normals",
        airsim.ImageType.Infrared: "infrared"
    }.get(image_type, "unknown")

def main():
    rospy.init_node('airsim_image_publisher', anonymous=True)
    bridge = CvBridge()
    

    image_types = [
        airsim.ImageType.Scene,
        airsim.ImageType.DepthPlanar,
        airsim.ImageType.DepthPerspective,
        airsim.ImageType.DepthVis,
        4,
        airsim.ImageType.Segmentation,
        airsim.ImageType.SurfaceNormals,
        airsim.ImageType.Infrared
    ]

    camera_id = "0"  # ID de la cámara a utilizar. Ajusta según sea necesario.
    publishers = {}

    # Crear un publicador para cada tipo de imagen
    for image_type in image_types:
        topic_name = f"/test_airsim/camera_id_{camera_id}/{get_image_type_name(image_type)}"
        publishers[image_type] = rospy.Publisher(topic_name, Image, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        try:
            responses = client.simGetImages([airsim.ImageRequest(camera_id, image_type, pixels_as_float=(image_type in [airsim.ImageType.DepthPerspective, airsim.ImageType.DepthPlanar, airsim.ImageType.DepthVis]), compress=False) for image_type in image_types])

            for response in responses:
                print(f"Image type: {get_image_type_name(response.image_type)}")
                print(f"Image width: {response.width}")
                print(f"Image height: {response.height}")

                if response.pixels_as_float:
                    img_array = np.array(response.image_data_float, dtype=np.float32)
                    img_array = img_array.reshape(response.height, response.width)
                    # Normalizar para visualización
                    img_array = np.clip(img_array, 0, 50)  # Ajusta el rango de profundidad
                    img_array = (img_array / 10 * 255).astype(np.uint8)  # Escala de 0 a 255
                    ros_image = bridge.cv2_to_imgmsg(img_array, encoding="mono8")
                else:
                    img_array = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                    if response.image_type == airsim.ImageType.Scene:
                        img_array = img_array.reshape(response.height, response.width, 3)
                        ros_image = bridge.cv2_to_imgmsg(img_array, encoding="bgr8")
                        
                        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                        img_rgb = img1d.reshape(response.height, response.width, 3)
                        rgb_message = bridge.cv2_to_imgmsg(img_rgb, encoding="bgr8")
    
                    elif response.image_type == airsim.ImageType.Segmentation:
                        img_array = img_array.reshape(response.height, response.width, 1)  # Cambia esto si Segmentation tiene más de un canal
                        ros_image = bridge.cv2_to_imgmsg(img_array, encoding="mono8")
                    elif response.image_type == airsim.ImageType.Infrared:
                        img_array = img_array.reshape(response.height, response.width, 1)
                        ros_image = bridge.cv2_to_imgmsg(img_array, encoding="mono8")
                    elif response.image_type == airsim.ImageType.SurfaceNormals:
                        img_array = img_array.reshape(response.height, response.width, 3)
                        ros_image = bridge.cv2_to_imgmsg(img_array, encoding="bgr8")  # Ajusta según cómo quieras visualizar los normales
                    else:
                        rospy.logwarn(f"Unhandled image type {response.image_type}")
                        continue  # Salta si el tipo de imagen no está manejado

                publishers[response.image_type].publish(ros_image)
        except Exception as e:
            rospy.logerr(f"Error getting images: {str(e)}")
        
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass