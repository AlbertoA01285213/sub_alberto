# sub_alberto
 
Este es mi copia del repositorio del submarino. Sirve para tener una idea de que hacer, como funciona y a donde ir. Sientete en confianza de clonar el repositorio y probarlo.


## Como correrlo
Aun no lo perfecciono, hay algunas cosas que hacen referencia a mi computadora, mas que nada las direcciones con los parametros. La primera modificacion que debes de hacer es en: 
<br>
<br>
*/sub_alberto/src/uuv_visualization/config/object_visualizer_params.yaml*
<br>
<br>
En este cambias la direccion a la de tu computadora, simplemente es redireccionar el root.
<br>
<br>
Similarmente haces lo mismo con el config en uuv_navigation llamado *bezier_params.yaml* y en el nodo de *line trayectory*

El launch oficial aun no esta en uuv_master. ahora el launch grande esta en el paquete de **uuv_visualization**. Por lo tanto el comando es:
```bash
ros2 launch uuv_navigation launch.py
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

