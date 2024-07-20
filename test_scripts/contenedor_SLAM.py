#!/usr/bin/env python3

import os
import subprocess
import signal
import sys

# Cambiar al directorio que contiene move_dron.py
os.chdir('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts')

# Función para manejar la señal de interrupción
def signal_handler(sig, frame):
    print("Stoping scripts")
    if process1:
        process1.terminate()
        process1.wait()  # Asegúrate de que el proceso se haya detenido correctamente
    if process2:
        process2.terminate()
        process2.wait()  # Asegúrate de que el proceso se haya detenido correctamente
    sys.exit(0)

# Registrar el manejador de señales para la interrupción
signal.signal(signal.SIGINT, signal_handler)  # Usar SIGINT para Ctrl+C

# Ejecutar los scripts
process1 = subprocess.Popen(["python3", "move_dron.py"])
process2 = subprocess.Popen(["python3", "publish_topics.py"])

# Mantener el script corriendo
try:
    while True:
        signal.pause()
except KeyboardInterrupt:
    pass  # Captura el Ctrl+C aquí para que no cierre el script
