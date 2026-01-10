# sub_alberto
 
Este es mi copia del repositorio del submarino. Sirve para tener una idea de que hacer, como funciona y a donde ir. Sientete en confianza de clonar el repositorio y probarlo.


## Como correrlo
Aun no lo perfecciono, hay algunas cosas que hacen referencia a mi computadora, mas que nada las direcciones con los parametros. Es decir, busca donde diga algo como /home/alberto/... y cambia 'alberto' a tu username. La primera modificacion que debes de hacer es en: 
<br>
<br>
*/sub_alberto/src/uuv_visualization/config/object_visualizer_params.yaml*
*/sub_alberto/src/uuv_navigation/scripts/line_trayectory.yaml*
<br>
<br>
En este cambias la direccion a la de tu computadora, simplemente es redireccionar el root.
<br>
<br>
Similarmente haces lo mismo con el config en uuv_navigation llamado *bezier_params.yaml*

Agrega dos carpetas vacías dentro de **uuv_vision**: una que se llame *database* y otra *scripts*

Ademas ocupas las siguientes librerias: pip install fastapi uvicorn opencv-python cv_bridge

pip install "opencv-python<4.9.0" "numpy<2.0"
pip install websockets

El launch oficial aun no esta en uuv_master. Ahora el launch grande esta en el paquete de **uuv_visualization**. Por lo tanto el comando es:
```bash
ros2 launch uuv_navigation gazebo_launch.py
```
Por default se corre con el mission file 2. Estos archivos se encuentran en **uuv_mission**, y son algoritmos que marcan pasos que seguirá el sub en misiones específicas. Por ahora son sólo pruebas, aunque en la simulación sí se pueden ver las estaciones de las misiones. Para correr la simulación con otro archivo:
```bash
ros2 launch uuv_navigation gazebo_launch.py mission_file:=mission_x.yaml
```
y sustituye *x* por el número de la misión del nombre del archivo. 

Se agrego una adaptacion para gazebo. Esto para poder incluir la camara a los calculos. No funciona del todo bien pero de algo sirve. Para correrlo:
```bash
pkill -9 ruby && pkill -9 parameter_brid && pkill -9 static_transform
ros2 launch uuv_visualization gazebo_launch.py mission_file:=mission_test.yaml
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

