# Escuela de Invierno de Robótica 2019-2020
## Control y Simulación de un Coche Robótico
## Modelado, Control y Simulación de Robots Móviles

En este proyecto se implementa:
* El modelo cinemático de un coche robótico
* Ley de control para mover el robot:
  * Move to Point
  * Follow Path
  * Move to Pose
  * Follow Line

Los archivos de arranque contienen un machote de código con la estructura mínima de clases dentro de un workspace de ROS. 

### Archivos de arranque
Los archivos se pueden obtener en la URL listada en el botón verde etiquetado "Clone or Download"

```$ git clone [URL]```

Lo que obtienes es:
* auto_driver/src:
  * 01_open_loop_driver.cpp - implementa un control a lazo abierto
  * 02_pose_estimation.cpp - implementa la estimación de la pose del robot utilizando integración numérica
  * 03_robot.cpp - clase Robot. Aquí es donde se implementan las funciones de actualización de estado y de control. 
    * Localiza las secciones entre los siguientes comentarios:
      ```
      // WRITE YOUR CODE: BEGIN
      ...
      // WRITE YOUR CODE: END
      ```
    * After you implement the methods, you also need to tune the constants in each controller to improve performance as much as possible
  * 03_controllers.cpp - this is the main program that uses the Robot class. 
  * 03_test.cpp - this program helps send paths to the controllers program without having to manually publish messages from the terminal. Feel free to try different paths
* auto_driver/CMakeLists.txt - compiling directives

### Compilación inicial
Después de obtener el código, ejecuta 
```
catkin_make
```
