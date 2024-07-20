#!/usr/bin/env python3
import airsim
import rospy

import os

def connectClient():
    global client  # Variable global para el cliente de AirSim
    
    ip = rospy.get_param('ip', 'Error')
    port = rospy.get_param('port', 'Error')
    
    ip = os.path.expandvars(ip)
    port = int(port)
    
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

if __name__ == '__main__':
    rospy.init_node('drone_connection_node', anonymous=True)  # Inicializar nodo de ROS
    connectAPI()  # Conectar a la API del simulador
    client.confirmConnection()
