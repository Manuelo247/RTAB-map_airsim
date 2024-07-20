#!/usr/bin/env python3
import rospy

from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, CameraInfo  # Asegúrate de importar CameraInfo
from nav_msgs.msg import Odometry
import tf

from cv_bridge import CvBridge
import signal
import sys

try:
    from drone_slam_simulation.connect_api import connectAPI
    from drone_slam_simulation.get_topics import get_cameraInfo, get_image, get_odom, get_odomInfo, init_varibles
except ImportError as e:
    rospy.logerr(f"ImportError: {e}")
    sys.exit(1)
    
def signal_handler(sig, frame):
    print("Stop publishing topics")
    sys.exit(0)

# Registrar el manejador de señales para la interrupción (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

def publishReadTopics():
    # Se definen los publicadores para los diferentes tópicos de ROS
    rgb_pub = rospy.Publisher('/airsim/image/rgb', Image, queue_size=10)
    depth_pub = rospy.Publisher('/airsim/image/depth', Image, queue_size=10)
    info_pub = rospy.Publisher('/airsim/image/camera_info', CameraInfo, queue_size=10)  # Publicador para CameraInfo
    odom_pub = rospy.Publisher('/airsim/odom', Odometry, queue_size=10)
    odomInfo_pub = rospy.Publisher('/airsim/odom_info', OdomInfo, queue_size=10)
    
    tf_broadcaster = tf.TransformBroadcaster()  # Inicializa un objeto para transmitir transformaciones
    bridge = CvBridge()
    init_varibles(client, tf_broadcaster, bridge)
    
    rate = rospy.Rate(10)  # Configura la tasa de publicación a 10 Hz
    
    
    while not rospy.is_shutdown():
        # Captura imágenes RGB y de profundidad, y obtiene los datos de odometría y odometría adicional
        rgb_message, depth_message, size = get_image()
        info_camera_message = get_cameraInfo(size)
        odom_message = get_odom()
        odom_info_message = get_odomInfo()

        current_time = rospy.Time.now()  # Obtiene el tiempo actual para las cabeceras de los mensajes

        # Asigna el tiempo actual a las cabeceras de los mensajes
        rgb_message.header.frame_id = "camera_link"  # Frame de la cámara
        rgb_message.header.stamp = current_time
        depth_message.header.frame_id = "camera_link"  # Frame de la cámara
        depth_message.header.stamp = current_time
        info_camera_message.header.frame_id = "camera_link"
        info_camera_message.header.stamp = current_time
        
        odom_message.header.frame_id = "odom"
        odom_message.header.stamp = current_time
        odom_info_message.header.frame_id = "odom"
        odom_info_message.header.stamp = current_time

        # Publica los mensajes en los tópicos correspondientes
        rgb_pub.publish(rgb_message)
        depth_pub.publish(depth_message)
        info_pub.publish(info_camera_message)
        odom_pub.publish(odom_message)
        odomInfo_pub.publish(odom_info_message)

        rate.sleep()  # Espera hasta el siguiente ciclo de publicación


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
    rospy.init_node('publish_topics', anonymous=True)
    initTopics()
