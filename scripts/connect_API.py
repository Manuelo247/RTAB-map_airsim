#!/usr/bin/env python3

import airsim
import rospy
import configparser
import os

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
    client.confirmConnection()

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
