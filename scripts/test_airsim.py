import airsim
import numpy as np
import os
import configparser

# Leer la configuración
config = configparser.ConfigParser()
config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')

# Obtener los valores de IP y puerto
ip = config['airsim']['ip']
ip = os.path.expandvars(ip)
port = int(config['airsim']['port'])

# Crear un cliente para conectar con el simulador
client = airsim.MultirotorClient(ip=ip, port=port)
# client.confirmConnection()
print("Codigo de movimiento ejecutado\n")

# Habilitar el control de la API
client.enableApiControl(True)
client.armDisarm(True)

# Despegar el dron
client.takeoffAsync().join()

# Hacer que el dron se mueva entre dos posiciones y cambie la orientación
while True:
    # Mover a la primera posición
    yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=0)
    client.moveToPositionAsync(10, 0, -10, 5, yaw_mode=yaw_mode).join()
    airsim.time.sleep(1)  # Pausar para verificar el movimiento

    # Mover a la segunda posición
    yaw_mode = airsim.YawMode(is_rate=False, yaw_or_rate=180)
    client.moveToPositionAsync(0, 0, -10, 5, yaw_mode=yaw_mode).join()
    airsim.time.sleep(1)  # Pausar para verificar el movimiento
    
#Tomar imagen desde el dron
# responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)])
# response = responses[0]

# Convertir a formato OpenCV
# img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
# img_rgb = img1d.reshape(response.height, response.width, 3)

# Guardar la imagen
# airsim.write_png(os.path.normpath('image2.png'), img_rgb) 

# Aterrizar el dron
client.landAsync().join()

# Deshabilitar el control de la API
client.armDisarm(False)
client.enableApiControl(False)
