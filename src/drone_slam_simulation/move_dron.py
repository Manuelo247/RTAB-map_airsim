#!/usr/bin/env python3

import airsim
import rospy

import signal
import sys
import os

os.chdir('/home/manuelo247/catkin_ws/src/drone_slam_simulation/src/drone_slam_simulation')
try:
    from drone_slam_simulation.connect_API_src import connectAPI
except ImportError as e:
    rospy.logerr(f"ImportError: {e}")
    sys.exit(1)

def signal_handler(sig, frame):
    rospy.loginfo("Drone disconnected")
    if client:
        client.armDisarm(False)
        client.enableApiControl(False)
    sys.exit(0)

# Registrar el manejador de señales para la interrupción
signal.signal(signal.SIGINT, signal_handler)

def loopMove():
    client.takeoffAsync().join()
    
    while True:
        # Mover a la primera posición
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=0)
        client.moveToPositionAsync(10, 0, -10, 5, yaw_mode=yaw_mode).join()
        airsim.time.sleep(1)  # Pausar para verificar el movimiento

        # Mover a la segunda posición
        yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=180)
        client.moveToPositionAsync(0, 0, -10, 5, yaw_mode=yaw_mode).join()
        airsim.time.sleep(1)  # Pausar para verificar el movimiento
    
def initMove():
    global client
    client = connectAPI()
    if not client:
        rospy.logerr("Failed to connect to AirSim API")
        return
    else:
        rospy.loginfo("Moving drone")
    try:
        loopMove()
        rospy.spin()
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        if client:
            client.armDisarm(False)
            client.enableApiControl(False)
        rospy.loginfo("Dron disarmed")

if __name__ == '__main__':
    rospy.init_node('drone_movement_node', anonymous=True)
    initMove()
