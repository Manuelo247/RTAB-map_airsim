import airsim
import numpy as np
import time
from pynput import keyboard

# Crear un cliente para conectar con el simulador

client = airsim.MultirotorClient(ip="2.tcp.us-cal-1.ngrok.io", port=12720)

# Habilitar el control de la API
client.enableApiControl(True)
client.armDisarm(True)

# Despegar el dron
client.takeoffAsync().join()

# Velocidad del dron
velocity = 5

# Estado de las teclas
pressed_keys = {"w": False, "s": False, "a": False, "d": False, "q": False, "e": False}

# Funciones para manejar la entrada del teclado
def on_press(key):
    try:
        if key.char in pressed_keys:
            pressed_keys[key.char] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        if key.char in pressed_keys:
            pressed_keys[key.char] = False
    except AttributeError:
        pass

# Configurar el listener del teclado
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

try:
    while True:
        # Obtener el control actual basado en las teclas presionadas
        control_x = 0
        control_y = 0
        control_z = 0

        if pressed_keys["w"]:
            control_x = velocity
        elif pressed_keys["s"]:
            control_x = -velocity

        if pressed_keys["a"]:
            control_y = -velocity
        elif pressed_keys["d"]:
            control_y = velocity

        if pressed_keys["q"]:
            control_z = velocity
        elif pressed_keys["e"]:
            control_z = -velocity

        # Mover el dron basado en los controles
        client.moveByVelocityAsync(control_x, control_y, control_z, 0.1).join()
        time.sleep(0.1)
except KeyboardInterrupt:
    # Aterrizar el dron cuando se interrumpe el script
    client.landAsync().join()
    client.armDisarm(False)
    client.enableApiControl(False)
    print("Aterrizaje completado y control deshabilitado.")
