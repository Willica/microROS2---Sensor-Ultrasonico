// Librería de micro ROS y estándar
#include <micro_ros_arduino.h>
#include <stdio.h>
// Librerías rcl y rclc
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
// Librerías para los mensajes a ser enviados
#include <rosidl_runtime_c/string.h>
#include <std_msgs/msg/header.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/string.h>

// Creación de los publicadores
rcl_publisher_t publisher_Range;
rcl_publisher_t publisher_OnlyDistance;

// Declaración explícita de variables necesarias
rosidl_runtime_c__String frame_id;
std_msgs__msg__Header header;
sensor_msgs__msg__Range msgRange;
std_msgs__msg__String msgOnlyDistance;

rclc_executor_t executor_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Definición de macros
#define LED_PIN 2       // Pin del LED para señalización
#define TRIGGER 5       // Pin de disparo del sensor ultrasónico
#define ECHO 18         // Pin de eco del sensor ultrasónico

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}} // Macro para chequeo de errores
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){} } // Macro para chequeo de errores sin interrupción

// Variables necesarias para determinar distancia a partir del sensor ultrasónico
long pulse_time;   
long distance;

// Callback del temporizador (publicador)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time); // No se usa last_call_time en este callback
  
  if (timer != NULL) {
    // Publicación del mensaje en el tópico /ultrasonic_sensor_node_publisher_Range
    rcl_ret_t ret = rcl_publish(&publisher_Range, &msgRange, NULL);
    if (ret != RCL_RET_OK) { // Manejo de errores en la publicación
      digitalWrite(LED_PIN, HIGH); // Enciende el LED si ocurre un error
    }

    // Conversión de distancia de centímetros a metros
    msgRange.range = distance * 0.01; 

    // Creación de una cadena con el valor de la distancia
    char distance_str[50]; // Buffer para la cadena de distancia
    snprintf(distance_str, sizeof(distance_str), "Distance: %.2f m", msgRange.range); // Formateo de la distancia en una cadena

    // Asignación de memoria para la cadena en msgOnlyDistance
    msgOnlyDistance.data.data = (char*)malloc(strlen(distance_str) + 1); // Reserva memoria para la cadena y el carácter nulo
    snprintf(msgOnlyDistance.data.data, msgOnlyDistance.data.capacity, "%s", distance_str); // Copia la cadena formateada a msgOnlyDistance
    msgOnlyDistance.data.size = strlen(msgOnlyDistance.data.data); // Establece el tamaño de la cadena

    // Publicación del mensaje en el tópico /ultrasonic_sensor_node_publisher_OnlyDistance
    ret = rcl_publish(&publisher_OnlyDistance, &msgOnlyDistance, NULL); 
    if (ret != RCL_RET_OK) { // Manejo de errores en la publicación
      digitalWrite(LED_PIN, HIGH); // Enciende el LED si ocurre un error
    }
  }
}

// Configuración inicial del nodo
void setup() {
  set_microros_transports(); // Configura los transportes de micro-ROS

  pinMode(LED_PIN, OUTPUT); // Configura el pin del LED como salida
  pinMode(TRIGGER, OUTPUT); // Configura el pin de disparo como salida
  pinMode(ECHO, INPUT);     // Configura el pin de eco como entrada

  delay(2000); // Espera 2 segundos para la inicialización

  allocator = rcl_get_default_allocator(); // Obtiene el asignador de memoria predeterminado

  // Inicialización del soporte de micro-ROS
  rclc_support_init(&support, 0, NULL, &allocator);

  // Inicialización del nodo
  rclc_node_init_default(&node, "ultrasonic_sensor_node", "", &support);

  // Inicialización del publicador para mensajes de rango
  rclc_publisher_init_default(
    &publisher_Range,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "ultrasonic_sensor_node_publisher_Range"
  );

  // Inicialización del publicador para mensajes de distancia
  rclc_publisher_init_default(
    &publisher_OnlyDistance,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "ultrasonic_sensor_node_publisher_OnlyDistance"
  );

  // Configuración del temporizador para ejecutar el callback periódicamente
  const unsigned int timer_timeout = 1000; // 1 segundo
  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  );

  // Inicialización del ejecutor para manejar el publicador
  rclc_executor_init(&executor_pub, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor_pub, &timer);

  // Inicialización del identificador de marco (frame_id)
  frame_id.data = (char*)malloc(20 * sizeof(char)); // Reserva memoria para el identificador del marco
  snprintf(frame_id.data, 20, "sensor_link"); // Configura el identificador del marco
  frame_id.size = strlen(frame_id.data); // Establece el tamaño del identificador
  frame_id.capacity = 20; // Establece la capacidad de la memoria asignada

  // Configuración del encabezado del mensaje
  header.frame_id = frame_id;

  // Configuración del mensaje de rango
  msgRange.header = header; // Asocia el encabezado al mensaje
  msgRange.min_range = 0.01; // Establece el rango mínimo del sensor
  msgRange.max_range = 2.0;  // Establece el rango máximo del sensor
  msgRange.range = 0.05;     // Establece el valor inicial del rango
  msgRange.field_of_view = 25 * 3.14 / 180; // Establece el campo de visión del sensor en radianes

  // Configuración del mensaje de distancia
  msgOnlyDistance.data.data = (char*)malloc(50 * sizeof(char)); // Reserva memoria para el mensaje de distancia
  snprintf(msgOnlyDistance.data.data, 50, "Bienvenido"); // Inicializa el mensaje con un saludo
  msgOnlyDistance.data.size = strlen(msgOnlyDistance.data.data); // Establece el tamaño del mensaje
  msgOnlyDistance.data.capacity = 50; // Establece la capacidad de la memoria asignada
}

// Bucle principal
void loop() {
  delay(100); // Espera 100 milisegundos antes de ejecutar el siguiente ciclo

  RCSOFTCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100))); // Ejecuta el callback del temporizador y maneja errores

  // Lectura del sensor ultrasónico
  digitalWrite(TRIGGER, HIGH); // Activa el disparo del sensor
  delayMicroseconds(10); // Espera 10 microsegundos
  digitalWrite(TRIGGER, LOW); // Desactiva el disparo del sensor
  delayMicroseconds(10); // Espera 10 microsegundos

  // Calcula el tiempo y la distancia del sensor ultrasónico
  pulse_time = pulseIn(ECHO, HIGH, 40000UL); // Lee el tiempo del pulso del eco
  distance = pulse_time / 59; // Calcula la distancia en centímetros

  // Limita la distancia máxima
  if (distance > 126) {
    distance = 125; // Establece un límite máximo para la distancia
  }
}