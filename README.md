
## Índice

* [Comandos necesarios](#comandos-necesarios)
  * [Iniciar el ROS Master](#iniciar-el-ros-master)
  * [Configurar el Workspace](#configurar-el-workspace)
  * [Dar Permisos de Ejecución a un Archivo `.py`](#dar-permisos-de-ejecución-a-un-archivo-py)
  * [Ejecutar el Launcher del Proyecto](#ejecutar-el-launcher-del-proyecto)
  * [Ejecutar RTAB-Map Individualmente](#ejecutar-rtab-map-individualmente)
  * [Ejecutar RViz Individualmente para la Visualización de Datos](#ejecutar-rviz-individualmente-para-la-visualización-de-datos)
  * [Ver el Diagrama de Comunicación entre Nodos](#ver-el-diagrama-de-comunicación-entre-nodos)
* [Cambios necesarios](#cambios-necesarios)
  * [Conectar a la API](#conectar-a-la-api)
    * [Cambiar directorio](#cambiar-directorio)
    * [Usar IP y puerto por defecto](#usar-ip-y-puerto-por-defecto)
* [Aclaraciones](#aclaraciones)
* [Árbol del proyecto](#árbol-del-proyecto)

## Comandos necesarios

A continuación se detallan los comandos y sus propósitos para trabajar con ROS en tu proyecto.

### Iniciar el ROS Master

```bash
roscore
```

Inicia el ROS master.

### Configurar el Workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Inicializa y actualiza tu workspace en `~/catkin_ws` (O el nombre de tu workspace).

### Dar Permisos de Ejecución a un Archivo `.py`

```bash
chmod +x ~/catkin_ws/src/drone_slam_simulation/scripts/airsim_image_publisher.py
```

Da permisos de ejecución a tu archivo `airsim_image_publisher.py`.

### Ejecutar el launcher del proyecto

```bash
roslaunch drone_slam_simulation rtabmap_airsim.launch
```

Ejecuta el proyecto, incluyendo el movimiento del dron, la publicación de datos, y la visualización en `rviz` y `RTAB-Map`. Asegúrate de haber ejecutado **roscore** previamente para que el ROS master esté en funcionamiento.

### Ejecutar RTAB-Map individualmente

```bash
roslaunch rtabmap_ros rtabmap.launch
```

Ejecuta RTAB-Map para la navegación y mapeo. (No necesario si ejecutas el .launch)

### Ejecutar RViz individualmente para la Visualización de datos

```bash
rosrun rviz rviz
```

Ejecuta `rviz` para visualizar los datos de RTAB-Map. (No necesario si ejecutas el .launch)

### Ver el Diagrama de Comunicación entre Nodos

```bash
rqt_graph
```

Puedes ver un diagrama de comunicación entre nodos. (No necesario si ejecutas el .launch)

# Cambios necesarios

Aquí mencionaré todos los cambios necesarios en caso de que quieras correr el proyecto en tu propia computadora.

## Conectar a la API

Para conectar con la API de AirSim, encontrarás el archivo de [connect_API_src.py](https://github.com/Manuelo247/RTAB-map_airsim/blob/master/src/drone_slam_simulation/connect_API_src.py), que lee el archivo [config.ini](https://github.com/Manuelo247/RTAB-map_airsim/blob/master/scripts/config.ini). En este archivo, establecerás tu IP y puerto si quieres que sean personalizados.

### Cambiar directorio

Será necesario cambiar la ruta al archivo `config.ini` en tu computadora. Modifica la línea en `connect_API_src.py` de:

```python
12    config.read('/home/manuelo247/catkin_ws/src/drone_slam_simulation/scripts/config.ini')
```

a la ruta donde se encuentre `config.ini` en tu computadora.

### Usar IP y puerto por defecto

Si no deseas establecer una IP y puerto personalizados, cambia la línea en `airsim_wrapper.py` de:

```python
25    client = airsim.MultirotorClient(ip=ip, port=port)
```

a:

```python
25    client = airsim.MultirotorClient()
```

## Aclaraciones

Dentro de la carpeta de [scripts](https://github.com/Manuelo247/RTAB-map_airsim/tree/master/scripts) encontrarás únicamente scripts utilizados para el testeo del proyecto, además del `config.ini` en caso de querer usar una IP y puerto personalizados para la conexión con la API.

Los **códigos importantes** que se usan en la ejecución del proyecto se encuentran en [src/drone_slam_simulation](https://github.com/Manuelo247/RTAB-map_airsim/tree/master/src/drone_slam_simulation).

En la carpeta de [launch](https://github.com/Manuelo247/RTAB-map_airsim/tree/master/launch) se encuentra el archivo para **ejecutar** los componentes necesarios para el proyecto, puedes encontrar como ejecutarlo en [Ejecutar el Launcher del Proyecto](#ejecutar-el-launcher-del-proyecto).

Recuerda **instalar las librerías** necesarias para `rviz`, `rtabmap` y demás para que el proyecto funcione correctamente.

# Árbol del proyecto

Aquí tienes una visualización rápida de los archivos imprescindibles del proyecto:

```
drone_slam_simulation
├── CMakeLists.txt
├── README.md
├── config
│   └── rtabmap_default.yaml
├── launch
│   └── rtabmap_airsim.launch
├── package.xml
├── scripts
│   ├── airsim_wrapper.py
│   ├── basic_connection.py
│   ├── basic_connection_ngrok.py
│   ├── config.ini
│   ├── contenedor_SLAM.py
│   ├── execute_bash.sh
│   ├── tempCodeRunnerFile.py
│   ├── test.py
│   ├── test_airsim.py
│   ├── test_socket.py
│   └── time_real_control.py
├── setup.py
└── src
    └── drone_slam_simulation
        ├── __init__.py
        ├── connect_API_src.py
        ├── move_dron.py
        └── publish_topics.py
```
