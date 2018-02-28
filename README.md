# Práctica 1 - Carrera de robots
*Prácticas de la asignatura de Visión Artificial y Robótica.*

Para iniciar el simulador 3D Gazebo con el mapa y los robots:

```bash
catkin_make
source devel/setup.bash
roslaunch turtlebot_gazebo_multiple create_multi_robot.launch 
```

Para poder **robot1** mediante comandos utilizamos el paquete `send_velocity_commands`:

```bash
rosrun send_velocity_commands send_velocity_commands_node
