# Integración de sensor Ultrasonico mediante microROS2

El presente repositorio contiene todo lo necesario para integrar un sensor ultrasonico aplicando microROS2. Los requisitos para ejecutar todo de forma correcta son tener instalado lo siguiente:

- Arduino IDE.
- Librería Micro ROS Arduino.
- Driver ESP32 (DOIT ESP32 DEVKIT V1).
- Sensor Ultrasonico HC-SR04.
- RViz.
- Terminal o Máquina Virtual con Ubuntu 22.04 LTS.
- En la máquina virtual, tener instalado [python3-rosdep](https://github.com/micro-ROS/micro_ros_setup.git).

## Programación archivo ".ino"

La programación del archivo ".ino" incluye todo lo referente a la programación de los nodos en microROS. A grandes rasgos, se creó un nodo que viene a estar representado por el sensor ultrasonico y de este se despliegan dos publicadores donde:

- ultrasonic_sensor_node: Corresponde al nodo del cual se despliegan dos publicadores.
- ultrasonic_sensor_node_publisher_Range: Envía un mensaje del tipo Range el cual permite integrar el sensor ultrasonico con RViz.
- ultrasonic_sensor_node_publisher_OnlyDistance: Envía un mensaje que contiene solo la distancia del sensor al objeto.

La lectura de los datos funciona en base a un ejecutor el cual llama a la función [timer_callback](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/arduino_program/arduino_program.ino) que permite la actualización de los valores publicados.

## Pasos para la integración del sensor y visualización en RViz

### Primer paso

Compilar y cargar el archivo [arduino_program.ino](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/arduino_program/arduino_program.ino) en el ESP32 mediante conexión USB.

### Segundo paso

Realizar la conexión acorde al siguiente diagrama.
![Diagrama sin título](https://github.com/user-attachments/assets/240ad212-2a0f-4b90-99cb-37179ac265b3)

### Tercer paso

Abrir terminal WSL o Máquina Virtual con Ubuntu y habilitar el puerto al cual está conectado el ESP32. Para el caso de WSL se deben seguir los pasos del siguiente [enlace](https://learn.microsoft.com/es-mx/windows/wsl/connect-usb#attach-a-usb-device).

Los comandos a utilizar son:
- `usbipd list`: Detecta los dispositivos conectados por USB.
![image](https://github.com/user-attachments/assets/d49a7ce5-2bda-438f-bc72-82bbd4f14fbf)
- `usbipd attach --wsl --busid "BUSID"`: Este comando permite habilitar el puerto en la terminal WSL con Ubuntu, es importante que "BUSID" sea reemplazado con la ID que indica el usbipd list.

### Cuarto paso

En la terminal WSL lo primero que se debe hacer es ubicarse en la carpeta donde se haya clonado el siguiente repositorio [python3-rosdep](https://github.com/micro-ROS/micro_ros_setup.git) mediante el siguiente comando:

- `cd uros_ws2`: Este comando permite moverse a la carpeta donde fue clonado el repositorio. Es importante que la carpeta a la que se haga "cd" corresponda efectivamente a la carpeta donde fue clonado el repositorio.
- `source install/local_setup.sh`
- `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`: Permite correr el agente de micro ros para la ejecución del nodo y publicador creado. Es importante que ttyUSB0 se reemplace por el dispositivo USB al cual está conectado el ESP32.
![image](https://github.com/user-attachments/assets/5ff6b645-05b0-4f71-9892-cdf93f477816)

### Quinto paso

Se ejecutarán comandos para verificar la correcta recepción de los datos enviados por los publicadores mediante los siguientes comandos:
- `ros2 topic echo /ultrasonic_sensor_node_publisher_Range`
- `ros2 topic info /ultrasonic_sensor_subscriber_OnlyDistance`

Al ejecutar dichos comandos se debería visualizar lo siguiente:
![image](https://github.com/user-attachments/assets/f37a66a5-42dc-4ce8-ba81-38f674b9a369)

### Sexto paso

El archivo [ojo.urdf](https://github.com/Willica/microROS2-Sensor-Ultrasonico/blob/main/Ojo_URDF/ojo.urdf) se debe tener en una carpeta de la terminal WSL o Máquina Virtual. En base a ello se deben ejecutar los siguientes comandos:

- `cd ojo`: Dirigirse a la carpeta donde se encuentra ubicado el URDF.
- `ros2 run robot_state_publisher robot_state_publisher ojo.urdf`: Comando que permite la ejecución del publicador de estados para poder visualizar el robot en RViz.
- `rviz2`: Comando que permite lanzar RViz.

Para agregar la configuración ([config.rviz](https://github.com/Willica/microROS2-Sensor-Ultrasonico/tree/main/rviz_config)) debe hacer lo siguiente:
- Mover el archivo a la terminal o Máquina Virtual.
- Clickear en File -> Open Config -> Buscar el directorio -> Clickear el archivo .rviz.

Al hacer todo lo anterior se visualizará de la siguiente forma en RViz:
![image](https://github.com/user-attachments/assets/9ec93f36-9613-4351-aa5e-a798b82a770d)

Al ejecutar `rqt_graph` se visualiza la comunicación entre los nodos de la siguiente forma:
![Captura de pantalla 2024-08-03 002513](https://github.com/user-attachments/assets/dd793a70-984a-4cb5-8a48-f35b155fd7be)

## Video de demostración funcionamiento

El siguiente enlace https://www.youtube.com/watch?v=f4G8VMUO8RI contiene un video de demostración de lo anteriormente mencionado.
