import airsim

import numpy as np
import rospy

from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, CameraInfo  # Asegúrate de importar CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, TransformStamped
import tf2_ros

from cv_bridge import CvBridge

def get_odomInfo():

    odom_info_msg = OdomInfo()
    
    odom_info_msg.header.stamp = rospy.Time.now()
    odom_info_msg.header.frame_id = "odom"

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
    
    tf_broadcaster.sendTransform(
        (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
        (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        current_time,
        transform.child_frame_id,
        transform.header.frame_id
    )

    return cam_info_msg

def get_image():
    # Captura de imágenes RGB y profundidad
    responses = client.simGetImages([
        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False),
        airsim.ImageRequest("0", airsim.ImageType.DepthPerspective, True, False)
    ])
    rgb_response = responses[0]
    depth_response = responses[1]
    
    size = [rgb_response.width, rgb_response.height]

    # Convertir a formato OpenCV
    img1d = np.frombuffer(rgb_response.image_data_uint8, dtype=np.uint8)
    img_rgb = img1d.reshape(rgb_response.height, rgb_response.width, 3)

    if len(depth_response.image_data_float) > 0:
        depth_img_meters = airsim.list_to_2d_float_array(depth_response.image_data_float, 
                        depth_response.width, depth_response.height)
       
        depth_mm_16bit = np.clip(depth_img_meters * 1000, 16, 65535).astype(np.uint16)
        
        # rospy.loginfo(f"Depth Image Data (sample): {depth_img_meters[0, 0]}")
        
        # rospy.loginfo(f"Depth Image Min Value: {np.min(depth_img_meters)}")
        # rospy.loginfo(f"Depth Image Max Value: {np.max(depth_img_meters)}")
        
        # rospy.loginfo(f"RGB Image Size: {size[0]}x{size[1]}")
        # rospy.loginfo(f"Depth Image Size: {depth_mm_16bit.shape[1]}x{depth_mm_16bit.shape[0]}")


    else:
        rospy.logwarn("No depth data received.")
        depth_mm_16bit = np.zeros((rgb_response.height, rgb_response.width), dtype=np.float32)

    # Guardar imágenes (opcional)
    # airsim.write_png(os.path.normpath('image.png'), img_rgb)
    # np.save('depth_image.npy', depth_mm_16bit)

    # Convertir a formato de mensaje ROS y publicar imágenes
    rgb_message = bridge.cv2_to_imgmsg(img_rgb, encoding="bgr8")
    depth_message = bridge.cv2_to_imgmsg(depth_mm_16bit, encoding="16UC1")

    return rgb_message, depth_message, size

def get_odom():
    
    state = client.getMultirotorState()
    currentTime = rospy.Time.now()
    # Crear un mensaje de odometría
    odom = Odometry()
    odom.header.stamp = currentTime
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
    transform.header.stamp = currentTime
    transform.header.frame_id = "odom"
    transform.child_frame_id = "base_link"
    transform.transform.translation.x = odom.pose.pose.position.x
    transform.transform.translation.y = odom.pose.pose.position.y
    transform.transform.translation.z = odom.pose.pose.position.z
    transform.transform.rotation = odom.pose.pose.orientation
    
    tf_broadcaster.sendTransform(
        (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z),
        (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w),
        currentTime,
        transform.child_frame_id,
        transform.header.frame_id
    )
    
    return odom

def init_varibles(localClient, localTf_broadcaster, localBridge):
    global client, tf_broadcaster, bridge
    client = localClient
    tf_broadcaster = localTf_broadcaster
    bridge = localBridge
    

if __name__ == '__main__':
    rospy.init_node('airsim_topics', anonymous=True)
