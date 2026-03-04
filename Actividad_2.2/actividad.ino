#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h> // Librería para manejar Strings
#include <stdio.h>

// Macros para manejo de errores de micro-ROS
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// --- Pines y Configuración ---
#define variador  34  // Pin de entrada analógica al que está conectado el potenciómetro
#define PWM_PIN   26  // PWM para Puente H
#define In1       14  // Pin de salida digital 
#define In2       27  // Pin de salida digital
#define LED_PIN   2   // Pin del LED integrado (usado para mostrar errores de micro-ROS)

#define freq 5000     // Frecuencia de PWM
#define resolution 8  // Resolución de PWM 2^8 = 256
#define PWM1_Ch 0     // Canal de PWM

// --- Variables del Sistema ---
float voltaje = 0;
float duty = 0;
int pot = 0;
int pwm = 0;
float Vcc = 3.3;
bool motorEnMovimiento = false;
char estado_actual[20] = "Inactivo"; // Guarda el estado (Derecha, Izquierda, Inactivo)

// --- Entidades de micro-ROS ---
rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;

// 3 Publicadores
rcl_publisher_t pub_pwm;
rcl_publisher_t pub_voltaje;
rcl_publisher_t pub_estado;

// 1 Suscriptor
rcl_subscription_t sub_cmd;

// 1 Timer para publicar periódicamente
rcl_timer_t timer;

// Mensajes
std_msgs__msg__Float32 msg_pwm;
std_msgs__msg__Float32 msg_voltaje;
std_msgs__msg__String msg_estado;
std_msgs__msg__String msg_cmd;

// Buffers de memoria para los mensajes String
char buffer_estado[20];
char buffer_cmd[10];

// --- Funciones del Motor ---
void detener() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, LOW);
  ledcWrite(PWM1_Ch, 0);
  strcpy(estado_actual, "Inactivo");
  duty = 0;
  voltaje = 0;
}

void derecha() {
  digitalWrite(In1, HIGH);
  digitalWrite(In2, LOW);
  strcpy(estado_actual, "Derecha");
}

void izquierda() {
  digitalWrite(In1, LOW);
  digitalWrite(In2, HIGH);
  strcpy(estado_actual, "Izquierda");
}

void lecturaPWM() {
  pot = analogRead(variador);
  voltaje = pot * (Vcc / 4095.0); 
  duty = 100 * voltaje / Vcc;
  
  pwm = map(pot, 0, 4095, 0, 255);
  ledcWrite(PWM1_Ch, pwm);
}

// Bucle de error visual
void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100); 
  }
}

// --- Callback del Suscriptor (Espera la D, I o S) ---
void subscription_callback(const void * msgin) {  
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  
  // Verificamos que el mensaje no esté vacío
  if (msg->data.size > 0) {
    char opcion = msg->data.data[0]; // Tomamos la primera letra

    switch (opcion) {
      case 'D':
      case 'd':
        motorEnMovimiento = true;
        derecha();
        break;

      case 'I':
      case 'i':
        motorEnMovimiento = true;
        izquierda();
        break;

      case 'S':
      case 's':
        motorEnMovimiento = false;
        detener();
        break;
    }
  }
}

// --- Callback del Timer (Publica los datos a cierta frecuencia) ---
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    
    // Si el motor está en movimiento, calculamos PWM. Si no, variables a 0.
    if (motorEnMovimiento) {
      lecturaPWM();
    }

    // 1. Asignar y publicar PWM
    msg_pwm.data = duty;
    rcl_publish(&pub_pwm, &msg_pwm, NULL);

    // 2. Asignar y publicar Voltaje
    msg_voltaje.data = voltaje;
    rcl_publish(&pub_voltaje, &msg_voltaje, NULL);

    // 3. Asignar y publicar Estado
    msg_estado.data.data = buffer_estado;
    strcpy(msg_estado.data.data, estado_actual);
    msg_estado.data.size = strlen(msg_estado.data.data);
    rcl_publish(&pub_estado, &msg_estado, NULL);
  }
}

void setup() {
  set_microros_transports(); 
  
  pinMode(variador, INPUT);
  pinMode(PWM_PIN, OUTPUT); 
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT); 
  pinMode(LED_PIN, OUTPUT);
  
  digitalWrite(LED_PIN, HIGH); 
  
  ledcSetup(PWM1_Ch, freq, resolution);
  ledcAttachPin(PWM_PIN, PWM1_Ch);
  
  detener(); // Asegurarnos de que inicie apagado
  
  delay(2000); // Esperar que se establezca la conexión

  // --- Inicialización de micro-ROS ---
  allocator = rcl_get_default_allocator();

  // Asignar memoria al mensaje entrante tipo String (Crucial para no causar un crash)
  msg_cmd.data.data = buffer_cmd;
  msg_cmd.data.capacity = sizeof(buffer_cmd);
  msg_cmd.data.size = 0;

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  // Nodo
  RCCHECK(rclc_node_init_default(&node, "control_motor_node", "", &support));

  // Publicadores
  RCCHECK(rclc_publisher_init_default(&pub_pwm, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "duty_cycle"));
  RCCHECK(rclc_publisher_init_default(&pub_voltaje, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "voltaje"));
  RCCHECK(rclc_publisher_init_default(&pub_estado, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "motor_state"));

  // Suscriptor
  RCCHECK(rclc_subscription_init_default(&sub_cmd, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "cmd_direccion"));

  // Timer (100 ms = 10 Hz)
  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));

  // Executor (Necesita manejar 2 tareas: 1 Suscriptor + 1 Timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &sub_cmd, &msg_cmd, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(10); // Un delay más corto que en tu ejemplo de referencia
  // El executor procesa los callbacks del suscriptor y dispara el timer cuando toca
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  
}