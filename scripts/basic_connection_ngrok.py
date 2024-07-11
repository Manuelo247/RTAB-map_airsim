import airsim
import configparser

# Leer la configuración
config = configparser.ConfigParser()
config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')

# Obtener los valores de IP y puerto
ip = config['airsim']['ip']
port = int(config['airsim']['port'])

# Reemplaza en ip con la URL proporcionada por Ngrok, por ejemplo '0.tcp.us-cal-1.ngrok.io'
# Reemplaza en port con el puerto externo específico asignado por Ngrok, por ejemplo 13864
client = airsim.MultirotorClient(ip=ip, port=port)
client.confirmConnection()

print("Conexión establecida con AirSim a través de Ngrok.")
