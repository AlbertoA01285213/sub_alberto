# Sub Alberto
 
Este es mi copia del repositorio del submarino. Sirve para tener una idea de que hacer, como funciona y a donde ir. Sientete en confianza de clonar el repositorio y probarlo.


## Como correrlo
El repositorio ya es funcional. Contiene un modelo del submarino que es capaz de tomar decisiones de manera autonoma. Este busca los retos y navega hacia ellos. 

Para correr la simulacion, se ocupan varias cosas. 
- Ros 2 (humble)
- Gazebo 6 (Ignition)

Las librerias de python necesarias:
- Ultralytics
- Numpy (<2.0)
- websockets (No se, Yeina sabe)
- OpenCV

Una vez clonado el repositorio, para correrlo es sencillo. Los launchfiles se encuentran dentro del paquete **uuv_master**, ya que es donde este el nodo principal. Actualmente hay 2 launchfiles.

```bash
ros2 launch uuv_master gazebo_launch.py
```
Este corre el modo autonomo y mas avanzado del submarino.

```bash
ros2 launch uuv_master teleop_launch.py
```
Este corre el modo teleoperado del submarino. Sirve para tomar videos para vision.
<br>

Es posible que el gazebo no se despliegue correctamente cuando corras los launchfile. En ese caso el gazebo se vera negro y vas a tener que pararlo. En este caso es normal que se queden procesos secundarios corriendo. Por lo tanto tendras que pararlos.

```bash
pkill -9 ruby && pkill -9 parameter_brid && pkill -9 static_transform
```




Por default se corre con el mission file 2. Estos archivos se encuentran en **uuv_mission**, y son algoritmos que marcan pasos que seguirá el sub en misiones específicas. Por ahora son sólo pruebas, aunque en la simulación sí se pueden ver las estaciones de las misiones. Para correr la simulación con otro archivo:
```bash
ros2 launch uuv_navigation gazebo_launch.py mission_file:=mission_x.yaml
```
y sustituye *x* por el número de la misión del nombre del archivo. 

Se agrego una adaptacion para gazebo. Esto para poder incluir la camara a los calculos. No funciona del todo bien pero de algo sirve. Para correrlo:
```bash

```

Si se desea abrir la terminal y ver mensajes dentro del gazebo:
```bash
export IGN_PARTITION=uuv_sim
ign topic -l
ign topic -e -t /topico

```

Ademas, es posible que necesites instalar varias librerias como "eigen3" solo que no me acuerdo de los comandos, nomas hazle caso a lo que la terminal te diga. Si lo haces, dime que librerias ocupas instalar para anotarlas aqui pls.

## WiP
Esto aun es un work in progress. Por lo tanto, hay varias cosas que deben de cambiar. 
- Primero es que bezier agarra puntos aleatorios. En realidad el mission_handler gracias a las misiones, seran las que manden el waypoint a ir.
- El siguiente, es que el sub se ve fatal volando
- Debo de anadir formas a los waypoints
- Agregar un mission_handler y misiones
- Vision
- No se que mas poner, pero muchas cosas mas.


<!-- 
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/Documents/sub_alberto/src
ros2 run image_view video_recorder --ros-args -r image:=/camera/left

pip install fastapi uvicorn opencv-python cv_bridge
pip install "opencv-python<4.9.0" "numpy<2.0"
pip install websockets

ros2 launch uuv_visualization gazebo_launch.py mission_file:=mission_test.yaml
ros2 topic echo /pose --msg-type geometry_msgs/msg/Pose
yolo detect train data=data.yaml model=yolo11m-seg.pt epochs=75 imgsz=720 batch=8 workers=4

-->