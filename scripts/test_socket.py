import socket

host = "127.0.0.1"  # Dirección IP de AirSim en Windows
port = 41451  # Puerto de AirSim

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)  # Tiempo de espera en segundos
    s.connect((host, port))
    print(f"Conexión exitosa con {host}:{port}")
    s.close()
except Exception as e:
    print(f"No se pudo conectar a {host}:{port}: {e}")
