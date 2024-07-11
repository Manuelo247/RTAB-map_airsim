import airsim
import configparser
import os

# Leer la configuración
config = configparser.ConfigParser()
config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')

# Obtener los valores de IP y puerto
ip = config['airsim']['ip']
ip = os.path.expandvars(ip)
port = int(config['airsim']['port'])
print(ip)

# Crear el cliente AirSim
client = airsim.MultirotorClient(ip=ip, port=port)
try:
    client.confirmConnection()
    print("Conexión establecida con AirSim.")
except Exception as e:
    print("Error al conectar con AirSim:", e)
