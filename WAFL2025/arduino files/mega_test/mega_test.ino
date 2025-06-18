#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <WAFL2025/WheelSpeeds.h>
#include <WAFL2025/SteeringAngles.h>

ros::NodeHandle nh;

// Encoder pins
#define STEERING_ENCODER_A 2
#define STEERING_ENCODER_B 3
#define WHEEL_ENCODER_A 18
#define WHEEL_ENCODER_B 19

// Constants
const int STEERING_PPR = 600;
const int WHEEL_PPR = 600;
const float STEERING_GEAR_RATIO = 4.0;
const float MAX_STEERING_ANGLE = 90.0; // Corrected to match HOME_ANGLE
const float HOME_ANGLE = 130.0;
const float COUNTS_PER_RANGE = ((STEERING_PPR * 4.0) * STEERING_GEAR_RATIO * 0.5) / 3.25;
float MAX_LINEAR = 2;

// Motor control pins
#define LEFT_MOTOR_PWM 10
#define LEFT_MOTOR_DIR 8
#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIR 7
#define FRONT_STEER_DIR A4
#define FRONT_STEER_STEP A5
#define LIMIT_SWITCH 40 // Limit switch pin

// Global variables
volatile long steeringPos = 0;
volatile long wheelPulseCount = 0;
volatile int wheelDirection = 1;
long steeringZeroPos = 0;
float left_speed = 0.0, right_speed = 0.0;
float front_steer = 0.0;
float currentSteeringAngle = 0.0;

// State flags
bool steering_in_progress = false;
bool driving_period_active = false;

// Timing control
const float STEERING_TOLERANCE = 0.5;    // Degrees tolerance
const int MAX_STEPS_PER_LOOP = 50;       // Increased to match homing speed
const unsigned long DRIVING_DURATION = 1000;  // 1 second
unsigned long driving_start_time = 0;
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100;  // 100ms = 10Hz

// Encoder state tracking
volatile uint8_t steeringEncoderOldState = 0;
const int8_t lookupTable[] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

// ROS message
std_msgs::Float32MultiArray sensor_msg;
float sensor_data[4] = {0};

// Callback functions
void wheelSpeedsCallback(const WAFL2025::WheelSpeeds& msg) {
  left_speed = msg.left_speed;
  right_speed = msg.right_speed;
}

void steeringAnglesCallback(const WAFL2025::SteeringAngles& msg) {
  front_steer = msg.front_angle;
}

// ROS publishers/subscribers
ros::Publisher sensor_pub("/swerve/raw_data", &sensor_msg);
ros::Subscriber<WAFL2025::WheelSpeeds> wheel_sub("/wheel_speeds", &wheelSpeedsCallback);
ros::Subscriber<WAFL2025::SteeringAngles> steer_sub("/steering_angles", &steeringAnglesCallback);

// Steering angle calculation
float getSteeringAngle() {
  noInterrupts();
  long currentPos = steeringPos;
  interrupts();
  float angle = (currentPos - steeringZeroPos) * MAX_STEERING_ANGLE / COUNTS_PER_RANGE;
  return constrain(angle, -120, 120);
}

// Homing function
void homeSteering() {
  // Move to home position (+120 degrees)
  digitalWrite(FRONT_STEER_DIR, HIGH); // Towards +120
  unsigned long start_time = millis();
  while (digitalRead(LIMIT_SWITCH) == LOW && (millis() - start_time < 10000)) { // While not pressed
    digitalWrite(FRONT_STEER_STEP, HIGH);
    delayMicroseconds(200);
    digitalWrite(FRONT_STEER_STEP, LOW);
    delayMicroseconds(200);
  }
  
  if (digitalRead(LIMIT_SWITCH) == LOW) {
    nh.logerror("Homing timeout: Limit switch not reached");
    return;
  }

  // Set steeringZeroPos based on home position
  noInterrupts();
  long pos_home = steeringPos;
  interrupts();
  float offset = (HOME_ANGLE / MAX_STEERING_ANGLE) * COUNTS_PER_RANGE;
  steeringZeroPos = pos_home - offset;
  nh.loginfo("Homing complete: Zero position set");

  // Move back to angle zero
  const float TARGET_ANGLE = 0.0;
  int max_iterations = 1000; // Prevent infinite loop
  for (int iter = 0; iter < max_iterations; iter++) {
    float currentAngle = getSteeringAngle();
    float error = TARGET_ANGLE - currentAngle;
    if (fabs(error) <= STEERING_TOLERANCE) {
      break;
    }
    digitalWrite(FRONT_STEER_DIR, error > 0 ? HIGH : LOW);
    for (int i = 0; i < MAX_STEPS_PER_LOOP; i++) {
      digitalWrite(FRONT_STEER_STEP, HIGH);
      delayMicroseconds(200);
      digitalWrite(FRONT_STEER_STEP, LOW);
      delayMicroseconds(200);
    }
  }
  nh.loginfo("Moved to zero position");
}

// Steering control
void updateSteering() {
  float targetAngle = front_steer * 180.0 / PI; // Convert to degrees
  float currentAngle = getSteeringAngle();
  float error = targetAngle - currentAngle;
  
  if (fabs(error) > STEERING_TOLERANCE) {
    steering_in_progress = true;
    digitalWrite(FRONT_STEER_DIR, error > 0 ? HIGH : LOW);
    
    for (int i = 0; i < MAX_STEPS_PER_LOOP; i++) {
      digitalWrite(FRONT_STEER_STEP, HIGH);
      delayMicroseconds(200);
      digitalWrite(FRONT_STEER_STEP, LOW);
      delayMicroseconds(200);
    }
  } else {
    steering_in_progress = false;
    if (!driving_period_active) {
      driving_period_active = true;
      driving_start_time = millis();
    }
  }
}

// Encoder ISRs
void wheelEncoderISR() {
  bool A = digitalRead(WHEEL_ENCODER_A);
  bool B = digitalRead(WHEEL_ENCODER_B);
  wheelDirection = (A == B) ? 1 : -1;
  wheelPulseCount++;
}

void steeringEncoderISR() {
  uint8_t newState = (digitalRead(STEERING_ENCODER_A) << 1) | digitalRead(STEERING_ENCODER_B);
  uint8_t state = (steeringEncoderOldState << 2) | newState;
  steeringPos += lookupTable[state];
  steeringEncoderOldState = newState;
}

// Setup function
void setup() {
  nh.getHardware()->setBaud(19200);
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(wheel_sub);
  nh.subscribe(steer_sub);
  
  // Encoder setup
  pinMode(STEERING_ENCODER_A, INPUT_PULLUP);
  pinMode(STEERING_ENCODER_B, INPUT_PULLUP);
  pinMode(WHEEL_ENCODER_A, INPUT_PULLUP);
  pinMode(WHEEL_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_A), wheelEncoderISR, CHANGE);
  steeringEncoderOldState = (digitalRead(STEERING_ENCODER_A) << 1) | digitalRead(STEERING_ENCODER_B);
  attachInterrupt(digitalPinToInterrupt(STEERING_ENCODER_A), steeringEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STEERING_ENCODER_B), steeringEncoderISR, CHANGE);
  
  // Motor and stepper setup
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR, OUTPUT);
  pinMode(FRONT_STEER_DIR, OUTPUT);
  pinMode(FRONT_STEER_STEP, OUTPUT);
  
  // Limit switch setup
  pinMode(LIMIT_SWITCH, INPUT_PULLUP);
  
  // ROS message initialization
  sensor_msg.data = sensor_data;
  sensor_msg.data_length = 4;
  
  // Perform homing
  homeSteering();
}

// Main loop
void loop() {
  unsigned long currentTime = millis();
  
  // Sensor update and publishing
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    noInterrupts();
    long pulses = wheelPulseCount;
    wheelPulseCount = 0;
    int dir = wheelDirection;
    currentSteeringAngle = getSteeringAngle();
    interrupts();
    
    float rpm = (pulses * 60.0) / (WHEEL_PPR * (UPDATE_INTERVAL/1000.0));
    float rads = (2 * PI * rpm * dir) / 60.0;
    
    sensor_data[0] = rads;
    sensor_data[1] = rads;
    sensor_data[2] = 0.0;
    sensor_data[3] = currentSteeringAngle * PI / 180.0;
    sensor_pub.publish(&sensor_msg);
    
    lastUpdateTime = currentTime;
  }
  
  if (driving_period_active && (currentTime - driving_start_time >= DRIVING_DURATION)) {
    driving_period_active = false;
  }
  
  if (!driving_period_active) {
    updateSteering();
  }
  
  if (driving_period_active || !steering_in_progress) {
    int pwm_left = (int)constrain((fabs(left_speed) / MAX_LINEAR) * 255.0, 0, 70);
    analogWrite(LEFT_MOTOR_PWM, pwm_left);
    digitalWrite(LEFT_MOTOR_DIR, left_speed < 0 ? HIGH : LOW);
    
    int pwm_right = (int)constrain((fabs(right_speed) / MAX_LINEAR) * 255.0, 0, 70);
    analogWrite(RIGHT_MOTOR_PWM, pwm_right);
    digitalWrite(RIGHT_MOTOR_DIR, right_speed < 0 ? HIGH : LOW);
  } else {
    analogWrite(LEFT_MOTOR_PWM, 0);
    analogWrite(RIGHT_MOTOR_PWM, 0);
  }
  
  nh.spinOnce();
  
  delay(10);
}
