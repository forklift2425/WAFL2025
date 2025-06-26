#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <WAFL2025/WheelSpeeds.h>
#include <WAFL2025/SteeringAngles.h>

// ROS node handle
ros::NodeHandle nh;

// Steering Encoder Configuration
#define STEERING_ENCODER_A 18  // Interrupt 0 (Mega)
#define STEERING_ENCODER_B 19  // Interrupt 1 (Mega)

// Wheel Speed Encoder Configuration
#define WHEEL_ENCODER_A 2    // Interrupt 5 (Mega)
#define WHEEL_ENCODER_B 3    // Interrupt 4 (Mega)

// Encoder parameters
const int STEERING_PPR = 600;
const int WHEEL_PPR = 600;
const float STEERING_GEAR_RATIO = 4.0;
const float MAX_STEERING_ANGLE = 90.0;
float currentAngle=0;
const float MAX_LINEAR = 2.0; 

// Motor Control Pins
#define LEFT_MOTOR_PWM 10
#define LEFT_MOTOR_DIR 8
#define RIGHT_MOTOR_PWM 9
#define RIGHT_MOTOR_DIR 7
#define FRONT_STEER_DIR A4
#define FRONT_STEER_STEP A5

// Global variables
volatile long steeringPos = 0;
volatile long wheelPulseCount = 0;
volatile int wheelDirection = 1;    // +1 for CW, -1 for CCW
long steeringZeroPos = 0;
float left_speed = 0.0, right_speed = 0.0;
float front_steer = 0.0, rear_steer = 0.0;
int steering_flag = 1;

// Steering Encoder Quadrature Decoding
volatile uint8_t steeringEncoderOldState = 0;
const int8_t lookupTable[] = {0,1,-1,0, -1,0,0,1, 1,0,0,-1, 0,-1,1,0};

// Timing
unsigned long lastUpdateTime = 0;
const unsigned long UPDATE_INTERVAL = 100; // ms

// ROS Messages
std_msgs::Float32MultiArray sensor_msg;
float sensor_data[4] = {0}; // Global to prevent scope issues

// Function Prototypes
void wheelEncoderISR();
void steeringEncoderISR();
float getSteeringAngle();
void calibrateSteering();
void steerStepper(float error);

// Steering Angle Calculation (with correct formula)
float getSteeringAngle() {
    long currentPos;
    noInterrupts();
    currentPos = steeringPos;
    interrupts();
    
    const float countsPerRange = ((STEERING_PPR * 4.0) * STEERING_GEAR_RATIO * 0.5) / 3.69;
    float angle = (currentPos - steeringZeroPos) * MAX_STEERING_ANGLE / countsPerRange;
    return constrain(angle, -120, 120);
}

// Calibrate Steering Zero Position
void calibrateSteering() {
    noInterrupts();
    steeringZeroPos = steeringPos;
    interrupts();
    nh.loginfo("Steering Calibrated!");
}

// Subscriber Callbacks
void wheelSpeedsCallback(const WAFL2025::WheelSpeeds& msg) {
    left_speed = msg.left_speed;
    right_speed = msg.right_speed;
    steering_flag = (fabs(left_speed) > 0.1 || fabs(right_speed) > 0.1) ? 0 : 1;
}

void steeringAnglesCallback(const WAFL2025::SteeringAngles& msg) {
    front_steer = msg.front_angle;
    rear_steer = msg.rear_angle;
}

// Publishers and Subscribers
ros::Publisher sensor_pub("/swerve/raw_data", &sensor_msg);
ros::Subscriber<WAFL2025::WheelSpeeds> wheel_sub("/wheel_speeds", &wheelSpeedsCallback);
ros::Subscriber<WAFL2025::SteeringAngles> steer_sub("/steering_angles", &steeringAnglesCallback);

// Stepper Motor Control
void steerStepper(float error) {
    
    if(steering_flag == 1 && fabs(error) > 2.0) {
     //   int steps = KP * error * (600 / 360.0); // Steps per degree
        digitalWrite(FRONT_STEER_DIR, error > 0 ? HIGH : LOW);
        
        for(int i=0; i<200; i++) {
            digitalWrite(FRONT_STEER_STEP, HIGH);
            delayMicroseconds(400);
            digitalWrite(FRONT_STEER_STEP, LOW);
            delayMicroseconds(400);
        }
    }
}

// Interrupt Service Routines
void wheelEncoderISR() {
  bool A = digitalRead(WHEEL_ENCODER_A);
  bool B = digitalRead(WHEEL_ENCODER_B);

  if (A == B) {
    wheelDirection = 1;  // CW
  } else {
    wheelDirection = -1; // CCW
  }

  wheelPulseCount++;
}

void steeringEncoderISR() {
  uint8_t newState = (digitalRead(STEERING_ENCODER_A) << 1) | digitalRead(STEERING_ENCODER_B);
  uint8_t state = (steeringEncoderOldState << 2) | newState;
  steeringPos += lookupTable[state];
  steeringEncoderOldState = newState;
}

void setup() {
    // Encoder Setup
    pinMode(STEERING_ENCODER_A, INPUT_PULLUP);
    pinMode(STEERING_ENCODER_B, INPUT_PULLUP);
    pinMode(WHEEL_ENCODER_A, INPUT_PULLUP);
    pinMode(WHEEL_ENCODER_B, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(WHEEL_ENCODER_A), wheelEncoderISR, CHANGE);
    steeringEncoderOldState = (digitalRead(STEERING_ENCODER_A) << 1) | digitalRead(STEERING_ENCODER_B);
    attachInterrupt(digitalPinToInterrupt(STEERING_ENCODER_A), steeringEncoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_ENCODER_B), steeringEncoderISR, CHANGE);

    // Motor Control Setup
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(FRONT_STEER_DIR, OUTPUT);
    pinMode(FRONT_STEER_STEP, OUTPUT);

    // ROS Initialization
    nh.getHardware()->setBaud(19200);
    nh.initNode();
    nh.advertise(sensor_pub);
    nh.subscribe(wheel_sub);
    nh.subscribe(steer_sub);

    // Initialize sensor message
    sensor_msg.data = sensor_data;
    sensor_msg.data_length = 4;

    calibrateSteering(); // Initial calibration
}

void loop() {
    unsigned long currentTime = millis();
    
    if(currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
        // Calculate wheel speed
        noInterrupts();
        long pulses = wheelPulseCount;
        wheelPulseCount = 0;
        int dir = wheelDirection;
        currentAngle = getSteeringAngle();
        interrupts();

        float rpm = (pulses * 60.0) / (WHEEL_PPR * (UPDATE_INTERVAL/1000.0));
        float rads = (2 * PI * rpm * dir) / 60.0;
        
        // Update sensor data
        sensor_data[0] = rads;
        sensor_data[1] = rads;
        sensor_data[2] = 0;
        sensor_data[3] = - currentAngle * PI /180;
        
        sensor_pub.publish(&sensor_msg);
        lastUpdateTime = currentTime;

    }
    
    float targetAngle = front_steer * 180.0 / PI;
    float error = targetAngle - getSteeringAngle();

    steerStepper(error);

    // Motor control
    
    int pwm_left = (int)constrain( (fabs(left_speed) / MAX_LINEAR)*255.0, 0, 60 );
    analogWrite(LEFT_MOTOR_PWM, pwm_left);
     digitalWrite(LEFT_MOTOR_DIR, left_speed < 0 ? HIGH : LOW);
     
    int pwm_right = (int)constrain( (fabs(right_speed) / MAX_LINEAR)*255.0, 0, 60 );
    analogWrite(RIGHT_MOTOR_PWM, pwm_right);
    digitalWrite(RIGHT_MOTOR_DIR, right_speed < 0 ? HIGH : LOW);

    nh.spinOnce();
    delay(10);
}
