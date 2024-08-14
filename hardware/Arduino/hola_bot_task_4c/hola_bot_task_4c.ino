#include "wifi_creds.h"
#include "hb3_params.h"

// #############################
#define left_servo_pin 25    // #
#define right_servo_pin 26   // #
#define rear_servo_pin 27    // #
#define marker_servo_pin 33  // #
#define LED_BUILTIN 2        // #
// #############################

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <iostream>

void ota_connection() {

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PWD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  ArduinoOTA.setHostname(HOSTNAME);

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 2

#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { error_loop(); } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }


void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

typedef struct WheelForce {
  float left;
  float right;
  float rear;
} WheelForce;

// Declare a global instance of WheelForce
WheelForce force;
void cmd_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)msgin;
  force.left = msg->linear.x;
  force.right = msg->linear.y;
  force.rear = msg->linear.z;
}

void micro_ros_setup() {
  set_microros_wifi_transports(WIFI_SSID, WIFI_PWD, LAPTOP_IP_ADD, 8888);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_NAME));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmd_callback, ON_NEW_DATA));
}

#include <ESP32Servo.h>
#include <cmath>

Servo left_servo;
Servo right_servo;
Servo rear_servo;
Servo marker_servo;

void servo_init() {
  left_servo.attach(left_servo_pin);
  right_servo.attach(right_servo_pin);
  rear_servo.attach(rear_servo_pin);
  marker_servo.attach(marker_servo_pin);
}

void run_motors(float f_left, float f_right, float f_rear) {
  left_servo.write(f_left);
  right_servo.write(f_right);
  rear_servo.write(f_rear);
}

// #####################################################################################
void setup() {
  Serial.begin(115200);

  // OTA SETUP
  ota_connection();

  // MICRO ROS SETUP
  micro_ros_setup();

  // OUTPUTS
  servo_init();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // OTA
  ArduinoOTA.handle();

  // MICRO ROS
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  // Motors
  run_motors(force.left, force.right, force.rear);
}
