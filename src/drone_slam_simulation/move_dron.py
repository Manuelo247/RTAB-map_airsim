#!/usr/bin/env python3

import airsim
import rospy

import signal
import sys

try:
    from drone_slam_simulation.connect_API_src import connectAPI
except ImportError as e:
    rospy.logerr(f"ImportError: {e}")  # Error al importar la función connectAPI
    sys.exit(1)

def signal_handler(sig, frame):
    rospy.loginfo("Drone disconnected")
    if client:
        client.armDisarm(False)  # Desarmar el dron
        client.enableApiControl(False)  # Deshabilitar el control de la API
    sys.exit(0)

# Registrar el manejador de señales para la interrupción (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

def loopMove():
    client.takeoffAsync().join()  # Despegar el dron
    
    while True:
        # Mover a la primera posición
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=0)
        client.moveToPositionAsync(20, 0, -10, 5, yaw_mode=yaw_mode).join()
        airsim.time.sleep(1)  # Pausar para verificar el movimiento

        # Mover a la segunda posición
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=180)
        client.moveToPositionAsync(0, 0, -10, 5, yaw_mode=yaw_mode).join()
        airsim.time.sleep(1)  # Pausar para verificar el movimiento
    
def initMove():
    global client
    
    client = connectAPI()  # Conectar a la API de AirSim
    
    if not client:
        rospy.logerr("Failed to connect to AirSim API")  # Error al conectar con AirSim
        return
    else:
        rospy.loginfo("Moving drone")  # Mensaje de éxito
    
    try:
        loopMove()  # Ejecutar el bucle de movimiento
        rospy.spin()  # Mantener el nodo en funcionamiento
    except Exception as e:
        print(f"An unexpected error occurred: {e}")  # Manejo de errores inesperados
    finally:
        if client:
            client.armDisarm(False)  # Desarmar el dron
            client.enableApiControl(False)  # Deshabilitar el contro

if __name__ == '__main__':
    rospy.init_node('drone_movement_node', anonymous=True)  # Inicializar nodo de ROS
    initMove()  # Conectar a la API del simulador
