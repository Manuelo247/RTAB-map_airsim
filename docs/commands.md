# Comandos variados
```bash
explorer.exe .
    # Para abrir su proyecto de WSL actual en el Explorador de archivos de Windows.
code . 
    # Abre Visual Studio Code desde terminal
xeyes
    # Prueba si el display de wsl a Windows funciona
sudo updatedb
    # Actualizar base de datos para locate 
locate <archivo>
    # Busca un archivo en tu maquita linux
```
# Conectar airsim con WSL
```bash
	./ngrok tcp 41453
		# [POWERSHELL] Inicia un servidor para conectarse remotamente, estando en la ubicación del ejecutable
	ipconfig
		# [POWERSHELL] Puede ver la dirección IPv4 necesaria para hacer la conexión de AirSim
```

# ROS	
```bash
roscore 
    #Inicia el ROS master

cd ~/catkin_ws
catkin_make
source devel/setup.bash
    # Inicializa y actualiza un wokspace

chmod +x ~/catkin_ws/src/drone_slam_simulation/scripts/airsim_image_publisher.py
    # Da permisos de ejecución a tu archivo .py

roslaunch drone_slam_simulation rtabmap_airsim.launch rtabmap:=true rviz:=true rtabmap_viz:=true
    # Ejecuta mi proyecto junto a rtabmap y rviz

roslaunch rtabmap_ros rtabmap.launch
    # Ejecuta RTAB-Map

rosrun rviz rviz
    # Ejecuta rviz para la visualización de RTAB-Map

rqt_graph
    # Puedes ver un diagrama de comunicación entre nodos

rosbag info example.bag
    # Ejecuta un bagfile para pruebas con RTAB-Map
```