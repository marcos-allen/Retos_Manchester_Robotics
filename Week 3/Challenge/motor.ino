// Include Libraries to be used
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <stdio.h>
#include <math.h> //

rcl_node_t node;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
std_msgs__msg__Float32 msg;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Configuración de Pines del Puente H
#define LED_PIN 23 
#define ENA_PIN 26   // Pin PWM para Velocidad
#define IN1_PIN 14  // Dirección 1 
#define IN2_PIN 27  // Dirección 2 

// Configuración PWM ENA
#define PWM_FRQ 980   //980 Hz
#define PWM_RES 8  
#define PWM_CHNL 0    
#define MSG_MIN_VAL -1.0 
#define MSG_MAX_VAL 1.0 

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100); 
  }
}

// Callback de Suscripción
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Float32 * msg = (const std_msgs__msg__Float32 *)msgin;
  
  // Restringimos el valor entre -1.0 y 1.0
  float set_point = constrain(msg->data, MSG_MIN_VAL, MSG_MAX_VAL);
  
  //Control de Dirección (IN1 e IN2)
  if (set_point > 0) {
    // Girar hacia adelante
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else if (set_point < 0) {
    // Girar en reversa
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  } else {
    // Detener el motor si el valor es 0
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  }

  //Control de Velocidad (ENA)
  float speed_abs = fabs(set_point); 
  uint32_t duty_cycle = (uint32_t) ((pow(2, PWM_RES) - 1) * speed_abs);
  
  ledcWrite(PWM_CHNL, duty_cycle);
}

void setup() {
  set_microros_transports(); 
  
  // Setup de pines digitales
  pinMode(LED_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 
  
  // Asegurarnos de que el motor inicie apagado
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  // Setup del PWM solo para el pin ENA
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  
  ledcAttachPin(ENA_PIN, PWM_CHNL);         
  
  delay(2000);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  
  //node motor

  RCCHECK(rclc_node_init_default(&node, "motor", "", &support));

  //  topic"cmd_pwm_team"
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm_team"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));  
}