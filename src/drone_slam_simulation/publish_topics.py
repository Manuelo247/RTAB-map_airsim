#!/usr/bin/env python3

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
import sys

os.chdir('/home/manuelo247/catkin_ws/src/drone_slam_simulation/src/drone_slam_simulation')
try:
    from drone_slam_simulation.connect_API_src import connectAPI
except ImportError as e:
    rospy.logerr(f"ImportError: {e}")
    sys.exit(1)

def get_odomInfo():

    odom_info_msg = OdomInfo()
    
    # Rellenar los campos necesarios
    # odom_info_msg.header = Header()
    odom_info_msg.header.stamp = rospy.Time.now()
    odom_info_msg.header.frame_id = "odom"

    # Ejemplo de datos (debes ajustar esto a tu configuración y datos reales)
    odom_info_msg.lost = False
    odom_info_msg.matches = 100
    odom_info_msg.inliers = 80
    odom_info_msg.icpInliersRatio = 0.8
    odom_info_msg.icpRotation = 0.1
    odom_info_msg.icpTranslation = 0.1
    odom_info_msg.icpStructuralComplexity = 0.5
    odom_info_msg.icpStructuralDistribution = 0.5
    odom_info_msg.icpCorrespondences = 50
    odom_info_msg.covariance = [0] * 36  # Ejemplo de matriz de covarianza
    odom_info_msg.features = 200
    odom_info_msg.localMapSize = 50
    odom_info_msg.localScanMapSize = 50
    odom_info_msg.localKeyFrames = 10
    odom_info_msg.localBundleOutliers = 5
    odom_info_msg.localBundleConstraints = 20
    odom_info_msg.localBundleTime = 0.01
    odom_info_msg.keyFrameAdded = True
    odom_info_msg.timeEstimation = 0.05
    odom_info_msg.timeParticleFiltering = 0.02
    odom_info_msg.stamp = rospy.Time.now().to_sec()
    odom_info_msg.interval = 0.1
    odom_info_msg.distanceTravelled = 1.0
    odom_info_msg.memoryUsage = 150  # en MB
    odom_info_msg.gravityRollError = 0.01
    odom_info_msg.gravityPitchError = 0.01

    # Transformaciones (ajustar según tu configuración)
    odom_info_msg.transform = Transform()
    odom_info_msg.transform.rotation.w = 1.0
    odom_info_msg.transformFiltered = Transform()
    odom_info_msg.transformFiltered.rotation.w = 1.0
    odom_info_msg.transformGroundTruth = Transform()
    odom_info_msg.transformGroundTruth.rotation.w = 1.0
    odom_info_msg.guess = Transform()
    odom_info_msg.guess.rotation.w = 1.0

    return odom_info_msg

def get_cameraInfo():
    camera_info = client.simGetCameraInfo("0")

    cam_info_msg = CameraInfo()
    cam_info_msg.header.seq = 0  # Placeholder
    cam_info_msg.header.stamp = rospy.Time.now()  # Usar el tiempo actual
    cam_info_msg.header.frame_id = '0'
    cam_info_msg.height = 1440  # Ajusta según sea necesario
    cam_info_msg.width = 2560  # Ajusta según sea necesario
    cam_info_msg.distortion_model = 'plumb_bob'  # Modelo típico de distorsión
    cam_info_msg.D = [0, 0, 0, 0, 0]  # Parámetros de distorsión (k1, k2, t1, t2, k3)
    cam_info_msg.K = [  # Matriz intrínseca de la cámara
        camera_info.fov, 0, camera_info.pose.position.x_val,
        0, camera_info.fov, camera_info.pose.position.y_val,
        0, 0, 1
    ]
    cam_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]  # Matriz de rotación (identidad aquí)
    cam_info_msg.P = [  # Matriz de proyección
        camera_info.fov, 0, camera_info.pose.position.x_val, 0,
        0, camera_info.fov, camera_info.pose.position.y_val, 0,
        0, 0, 1, 0
    ]
    cam_info_msg.binning_x = 0
    cam_info_msg.binning_y = 0
    cam_info_msg.roi.x_offset = 0
    cam_info_msg.roi.y_offset = 0
    cam_info_msg.roi.height = 0
    cam_info_msg.roi.width = 0
    cam_info_msg.roi.do_rectify = False

    return cam_info_msg

def get_image(bridge):
    # Captura de imágenes RGB y profundidad
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("0", airsim.ImageType.DepthPlanar, True, False)
    ])
    rgb_response = responses[0]
    depth_response = responses[1]

    # Convertir a formato OpenCV
    img1d = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(rgb_response.height, rgb_response.width, 3)

    if len(depth_response.image_data_float) > 0:
        depth1d = np.array(depth_response.image_data_float, dtype=np.float32)
        depth_img = depth1d.reshape(depth_response.height, depth_response.width)
    else:
        rospy.logwarn("No depth data received.")
        depth_img = np.zeros((rgb_response.height, rgb_response.width), dtype=np.float32)

    # Guardar imágenes (opcional)
    # airsim.write_png(os.path.normpath('image.png'), img_rgb)
    # np.save('depth_image.npy', depth_img)

    # Convertir a formato de mensaje ROS y publicar imágenes
    rgb_message = bridge.cv2_to_imgmsg(img_rgb, encoding="bgr8")
    depth_message = bridge.cv2_to_imgmsg(depth_img, encoding="32FC1")
    
    return rgb_message, depth_message

def get_odom(tf_broadcaster):
    
    state = client.getMultirotorState()
    # Crear un mensaje de odometría
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = "odom"
    odom.child_frame_id = "base_link"

    # Posición
    odom.pose.pose.position.x = state.kinematics_estimated.position.x_val
    odom.pose.pose.position.y = state.kinematics_estimated.position.y_val
    odom.pose.pose.position.z = - state.kinematics_estimated.position.z_val

    # Orientación (convertir quaternion de AirSim a ROS)
    orientation = state.kinematics_estimated.orientation
    odom.pose.pose.orientation.x = orientation.x_val
    odom.pose.pose.orientation.y = orientation.y_val
    odom.pose.pose.orientation.z = orientation.z_val
    odom.pose.pose.orientation.w = orientation.w_val

    # Velocidades lineales
    odom.twist.twist.linear.x = state.kinematics_estimated.linear_velocity.x_val
    odom.twist.twist.linear.y = state.kinematics_estimated.linear_velocity.y_val
    odom.twist.twist.linear.z = state.kinematics_estimated.linear_velocity.z_val

    # Velocidades angulares
    odom.twist.twist.angular.x = state.kinematics_estimated.angular_velocity.x_val
    odom.twist.twist.angular.y = state.kinematics_estimated.angular_velocity.y_val
    odom.twist.twist.angular.z = state.kinematics_estimated.angular_velocity.z_val
    
    transform = TransformStamped()
    transform.header.stamp = odom.header.stamp
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation

    tf_broadcaster.sendTransform(transform)
    
    return odom

# def publish_tf(br):
#     # Publicar transformaciones desde `odom` a `base_link`
#     br.sendTransform(
#         (0, 0, 0),  # La posición en `odom`
#         (0, 0, 0, 1),  # La orientación en `odom`
#         rospy.Time.now(),  # El tiempo actual
#         "base_link",  # El `child_frame_id`
#         "odom"  # El `parent_frame_id`
#     )
#     # Publicar transformaciones desde `camera_link` a `base_link`
#     br.sendTransform(
#         (0, 0, 0),  # La posición en `camera_link`
#         (0, 0, 0, 1),  # La orientación en `camera_link`
#         rospy.Time.now(),  # El tiempo actual
#         "camera_link",  # El `child_frame_id`
#         "base_link"  # El `parent_frame_id`
#     )


def publishReadTopics():
    rgb_pub = rospy.Publisher('/airsim/image_raw', Image, queue_size=10)
    depth_pub = rospy.Publisher('/airsim/depth', Image, queue_size=10)
    info_pub = rospy.Publisher('/airsim/camera_info', CameraInfo, queue_size=10)  # Publicador para CameraInfo
    odom_pub = rospy.Publisher('/airsim/odom', Odometry, queue_size=10)
    odomInfo_pub = rospy.Publisher('/airsim/odom_info', OdomInfo, queue_size=10)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    bridge = CvBridge()
    
    info_message = get_cameraInfo()

    rate = rospy.Rate(10)  # Publicar a 10Hz

    # br = tf.TransformBroadcaster()
    
    while not rospy.is_shutdown():
    
        rgb_message, depth_message = get_image(bridge)
        odom = get_odom(tf_broadcaster)
        odom_info_msg = get_odomInfo()
        
        # publish_tf(br)
        
        current_time = rospy.Time.now()
        rgb_message.header.stamp = current_time
        depth_message.header.stamp = current_time
        info_message.header.stamp = current_time
        # odom_info_msg.header.stamp = current_time

        rgb_pub.publish(rgb_message)
        depth_pub.publish(depth_message)
        odom_pub.publish(odom)
        info_pub.publish(info_message)
        odomInfo_pub.publish(odom_info_msg)

        rate.sleep()

def initTopics():
    global client 
    client = connectAPI()
    if not client:
        rospy.logerr("Failed to connect to AirSim API")
        return
    else:
        rospy.loginfo("Publishing topic")
    
    try:
        publishReadTopics()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interruption: Stopping topics publishing")
    except KeyboardInterrupt:
        rospy.logerr("Interrupción capturada, cerrando...")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
    finally:
        # Asegúrate de que client esté disponible antes de llamar a métodos
        rospy.logerr("Failed publishing topics or interrupted")


if __name__ == '__main__':
    rospy.init_node('airsim_image_publisher', anonymous=True)
    initTopics()
