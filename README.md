# Integración de sensor Ultrasonico mediante microROS2
El presente repositorio contiene todo lo necesario para poder integrar un sesnor ultrasonico aplicando microROS2, los requisitos para poder ejecutar todo de forma correcta son tener instalado lo siguiente.
- Arduino IDE.
- Libreria Micro ROS Arduino.
- Driver ESP32 (DOIT ESP32 DEVKIT V1).
- Sensor Ultrasonico HC-SR04.
- RViz.
- Terminal o Maquina Virtual con Ubuntu 22.04 LTS.
- En la maquina virtual tener instalado [python3-rosdep](https://github.com/micro-ROS/micro_ros_setup.git).

## Programación archivo ".ino".

La programación del archivo ".ino" incluye todo lo referente a la programación de los nodos en microROS. A grandes rasgos se creo un nodo que viene a estar representado por el sensor ultrasonico y de este se despliegan dos publicadores donde:

- ultrasonic_sensor_node: Corresponde al nodo del cual se despliegan dos publicadores.
- ultrasonic_sensor_node_publisher_Range: Envia un mensaje del tipo Range el cual permite integrar el sensor ultrasonico con RViz.
- ultrasonic_sensor_node_publisher_OnlyDistance: Envia un mensaje que contiene solo la distancia del sensor al objeto.

La lectura de los datos funciona en base a un ejecutor el cual llama a la función [timer_callback](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/arduino_program/arduino_program.ino) que permite la actualización de los valores publicados.

## Pasos para la integración del sensor y visualización en RViz.

### Primer paso.

Compilar y cargar el archivo [arduino_program.ino](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/arduino_program/arduino_program.ino) en el ESP32 mediante conexión USB.

#### Paso dos.

Realizar la conexión acorde al siguiente diagrama.
![Diagrama sin título](https://github.com/user-attachments/assets/240ad212-2a0f-4b90-99cb-37179ac265b3)

### Paso tres.

Abrir terminal WSL o Maquina virtual con ubuntu y habilitar el puerto al cual esta conectado el ESP32.
Para el caso de WSL se debe seguir los pasos del siguiente [enlace](https://learn.microsoft.com/es-mx/windows/wsl/connect-usb#attach-a-usb-device).

Los comandos a utilizar son:
- usbipd list: Detecta los dispositivos conectados por usb.
![image](https://github.com/user-attachments/assets/d49a7ce5-2bda-438f-bc72-82bbd4f14fbf)
- usbipd attach --wsl --busid "BUDID": Este comando permite habilitar el puerto en la terminal WSL con ubuntu, es importante que "BUSID" sea reemplazado con la ID que indica el usbipd list.

### Paso cuatro.

En la terminal WSL lo primero que se debe hacer es ubicarse en la carpeta donde se haya clonado el siguiente repositorio [python3-rosdep](https://github.com/micro-ROS/micro_ros_setup.git) mediante el siguietne comando.

- cd uros_ws2: Este comando permite moverse a la carpeta donde fue clonado el respositorio. Es importante que la carpeta a la que se haga "cd" corresponde efectivamente a la carpeta donde fue clonado el respositorio.
- source install/local_setup.sh
- ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0: Permite correr el agente de micro ros para la ejecución del nodo y publicador creado. Es importante que ttyUSB0 se reemplace por el dispositivo USB al cual esta conectado el ESP32.
![image](https://github.com/user-attachments/assets/5ff6b645-05b0-4f71-9892-cdf93f477816)

### Paso cinco.

Se ejecutaran comando para verificar la correcta recepción de los datos enviados por los publicadores mediante los siguiente comandos.
- ros2 topic echo /ultrasonic_sensor_node_publisher_Range
- ros2 topic info /ultrasonic_sensor_subscriber_OnlyDistance

Al ejecutar dichos comando se deberia visualziar lo siguiente:
![image](https://github.com/user-attachments/assets/f37a66a5-42dc-4ce8-ba81-38f674b9a369)

### Paso seis.

El archivo [ojo.urdf](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/Ojo_URDF/ojo.urdf) se debe tener en una carpeta de la terminal WSL o Maquina Virtual, en base a ello se deben ejecutar los siguientes comandos:

- cd ojo: Dirigirse a la carpeta donde se encuentra ubicado el URDF.
- ros2 run robot_state_publisher robot_state_publisher ojo.urdf: Comando que permite la ejecución del publicador de estados para poder visualziar el robot en RViz.
- rviz2: Comando que permite lanzar RViz.

Para agregar la configuración ([config.rviz](https://github.com/Willica/microROS2-Sensor-Ultrasonico/tree/main/rviz_config)) debe hacer lo siguiente.
- Mover el archivo a la terminal o Maquina Virtual.
- Clickear en File -> Open Config -> Buscar el directorio -> Clcikear el archivo .rviz.

Al hacer todo lo anterior se visualziara de la siguiente forma RViz:
![image](https://github.com/user-attachments/assets/9ec93f36-9613-4351-aa5e-a798b82a770d)

Al ejecutar rqt_graph se visualiza la comunicación entre los nodos de la siguiente forma.
![Captura de pantalla 2024-08-03 002513](https://github.com/user-attachments/assets/dd793a70-984a-4cb5-8a48-f35b155fd7be)


## Video de demostración funcionamiento.
El video mostrado a continuación permite visualziar el funcionamiento de lo descrito de mejor forma.

[![](https://markdown-videos.deta.dev/youtu.be/f4G8VMUO8RI)](youtu.be/f4G8VMUO8RI)

[![](https://markdown-videos.deta.dev/youtu.be/f4G8VMUO8RI?si=0PJyyWFF80HROmhx)](youtu.be/f4G8VMUO8RI?si=0PJyyWFF80HROmhx)

 

