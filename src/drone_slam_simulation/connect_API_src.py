#!/usr/bin/env python3

import airsim
import rospy
import configparser
import os


def readConfig():
    # Leer el archivo config.ini
    config = configparser.ConfigParser()
    config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')

    # Obtener los valores de IP y puerto
    ip = config['airsim']['ip']
    ip = os.path.expandvars(ip)
    port = int(config['airsim']['port'])

    return ip, port
    # Crear un cliente para conectar con el simulador

def connectClient():
    global client
    ip, port = readConfig()
    client = airsim.MultirotorClient(ip=ip, port=port)
    # client.confirmConnection()
    
def connectAPI():
    connectClient()
    # Habilitar el control de la API
    client.enableApiControl(True)
    client.armDisarm(True)
    
    return client
    
def disconnectAPI():
    #Deshabilitar el control de la API
    client.armDisarm(False)
    client.enableApiControl(False)
    
    
if __name__ == '__main__':
    rospy.init_node('drone_connection_node', anonymous=True)
    connectAPI()
