#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Custom message headers (make sure these are generated properly)
#include <WAFL2025/WheelSpeeds.h>
#include <WAFL2025/SteeringAngles.h>

// ROS Node
ros::NodeHandle nh;

// UART Configuration
#define STM32_BAUDRATE 9600

// Motor Control Pins
#define LEFT_MOTOR_PWM 5
#define LEFT_MOTOR_DIR 4
#define RIGHT_MOTOR_PWM 6
#define RIGHT_MOTOR_DIR 7

// Front Steering Pins (Stepper Motor)
#define FRONT_STEER_DIR 8
#define FRONT_STEER_STEP 9

// Rear Steering Pins (DC Motor + Potentiometer)
#define REAR_STEER_PWM 10
#define REAR_STEER_DIR_1 11
#define REAR_STEER_DIR_2 12

// Rear Potentiometer Pin
#define REAR_POT_PIN A0
const float ADC_TO_DEGREES = 3600.0 / 1023.0;  // 10 turns = 3600 degrees
float initialADC = 0;
float accu=0;
// Control Variables
float left_speed = 0.0;
float right_speed = 0.0;
float front_steer = 0.0;
float rear_steer = 0.0;

// UART Data Variables
uint8_t left_speed_byte = 0;
uint8_t right_speed_byte = 0;
uint8_t front_steer_byte = 0;

// ROS Messages
std_msgs::Float32MultiArray sensor_msg;
ros::Publisher sensor_pub("/swerve/raw_data", &sensor_msg);

// Subscribers
void wheelSpeedsCallback(const WAFL2025::WheelSpeeds& msg) {
  left_speed = msg.left_speed;
  right_speed = msg.right_speed;
}

void steeringAnglesCallback(const WAFL2025::SteeringAngles& msg) {
  front_steer = msg.front_angle;
  rear_steer = msg.rear_angle;
}

ros::Subscriber<WAFL2025::WheelSpeeds> wheel_sub("/wheel_speeds", &wheelSpeedsCallback);
ros::Subscriber<WAFL2025::SteeringAngles> steer_sub("/steering_angles", &steeringAnglesCallback);

void parseSTM32Data() {
  if (Serial.available() >= 4) {
    uint8_t start = Serial.read();
    if (start == 0xFF) {
      left_speed_byte = Serial.read();
      right_speed_byte = Serial.read();
      front_steer_byte = Serial.read();
      
      // Debug prints
      //Serial.print(left_speed_byte); Serial.print(" ");
      //Serial.print(right_speed_byte); Serial.print(" ");
      //Serial.println(front_steer_byte);
      //delay(120);
    }
  }
}

void moveFrontSteering(float front_error) {
  float desired_angle = front_error * 90.0;
  int steps_to_move = (int)(desired_angle * 0.107);
  digitalWrite(FRONT_STEER_DIR, front_error > 0 ? HIGH : LOW);

  for (int i = 0; i < abs(steps_to_move); i++) {
    digitalWrite(FRONT_STEER_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(FRONT_STEER_STEP, LOW);
    delayMicroseconds(500);
  }
}

void moveRearSteering(float rear_error) {
  digitalWrite(REAR_STEER_DIR_1, rear_error > 0 ? HIGH : LOW);
  digitalWrite(REAR_STEER_DIR_2, rear_error > 0 ? LOW : HIGH);
  int pwm = (int)(abs(rear_error) * 255);
  analogWrite(REAR_STEER_PWM, pwm);
}

void setup() {
  Serial.begin(STM32_BAUDRATE);
  
  // Pin setup
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(FRONT_STEER_DIR, OUTPUT);
  pinMode(FRONT_STEER_STEP, OUTPUT);
  pinMode(REAR_STEER_PWM, OUTPUT);
  pinMode(REAR_STEER_DIR_1, OUTPUT);
  pinMode(REAR_STEER_DIR_2, OUTPUT);
  pinMode(REAR_POT_PIN, INPUT);
  
  // Calibrate rear potentiometer
  // ROS initialization
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(wheel_sub);
  nh.subscribe(steer_sub);

  // Allocate memory for sensor message
  float sensor_data_array[4];  // Static array declaration
  sensor_msg.data = sensor_data_array;
  sensor_msg.data_length = 4;

  // Set initial position (zero point)
  initialADC = analogRead(REAR_POT_PIN);
  delay(100);  // Allow ADC to stabilize
}

void loop() {
  //parseSTM32Data();
  
  int currentADC = analogRead(REAR_POT_PIN);
  delay(100);
  
  // Calculate relative angle in degrees
  float relative_angle = (currentADC - initialADC) * ADC_TO_DEGREES * 3 /27;
  
  // Prepare and publish sensor message
  sensor_msg.data[0] = (float)left_speed_byte / 255.0;
  sensor_msg.data[1] = (float)right_speed_byte / 255.0;
  sensor_msg.data[2] = (float)front_steer_byte / 255.0;
  sensor_msg.data[3] = relative_angle;
  sensor_pub.publish(&sensor_msg);

  // Motor control
  digitalWrite(LEFT_MOTOR_DIR, left_speed > 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(left_speed) * 255);
  digitalWrite(RIGHT_MOTOR_DIR, right_speed > 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(right_speed) * 255);

  // Steering control
  float front_error = front_steer - sensor_msg.data[2];
  moveFrontSteering(front_error);
  
  float rear_error = rear_steer - (relative_angle / 3.0);
  moveRearSteering(rear_error);

  nh.spinOnce();
  delay(50);
}
