#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

// Custom message headers (make sure these are generated properly)
#include <WAFL2025/WheelSpeeds.h>
#include <WAFL2025/SteeringAngles.h>

// ROS node handle and publisher
ros::NodeHandle nh;
//std_msgs::Float32 angle_msg;
std_msgs::Float32MultiArray sensor_msg;
ros::Publisher sensor_pub("/swerve/raw_data", &sensor_msg);

// Potentiometer Front pin and initial position
const int potPin = A0;
// Stepper motor settings
const int microsteps = 8;                       // Using 1/8 microstepping
const int stepsPerRev = 200 * microsteps*4;     // 1600 steps/rev
const int microStep_delay = 400;                // Delay between microsteps
// Position tracking
long currentSteps = 0;  // Tracks current stepper position
float potValue = 0.0 ;
float currentAngle = 0.0 ;
long targetSteps = 0.0 ;
int intialpotValue=0.0;
int stepDelay=400;
// Potentiometer Rear pin and initial position
const int potPin2 = A5;
float acc=0;

#define STM32_BAUDRATE 9600

// Motor Control Pins
#define LEFT_MOTOR_PWM  9
#define LEFT_MOTOR_DIR  4
#define RIGHT_MOTOR_PWM 5
#define RIGHT_MOTOR_DIR 6

// Front Steering Pins (Stepper Motor)
#define FRONT_STEER_DIR A4
#define FRONT_STEER_STEP A5

// Rear Steering Pins (DC Motor)
#define REAR_STEER_PWM 12
#define REAR_STEER_DIR_1 8
#define REAR_STEER_DIR_2 13

#define ENCODER_PIN_A 2  // Interrupt pin
#define ENCODER_PIN_B 3  // Regular digital input

// Encoder parameters
const int PPR = 600;           // Pulses Per Revolution
const unsigned long UPDATE_INTERVAL = 100; // Update every 100 ms

// Variables
volatile long pulse_count = 0;
volatile int direction = 1;    // +1 for CW, -1 for CCW

unsigned long last_update_time = 0;
float rpm = 0.0;

// Control Variables
float left_speed = 0.0;
float right_speed = 0.0;
float front_steer = 0.0;
float rear_steer = 0.0;

// UART Data Variables
uint8_t left_speed_byte = 0;
uint8_t right_speed_byte = 0;
uint8_t front_steer_byte = 0;

int steering_flag =1;

// Subscribers
void wheelSpeedsCallback(const WAFL2025::WheelSpeeds& msg) {
  left_speed = msg.left_speed;
  right_speed = msg.right_speed;
  if(left_speed !=0 ||right_speed !=0)
  {
    steering_flag=0;
  }
  else
  {
    steering_flag=1;
  }
}

void steeringAnglesCallback(const WAFL2025::SteeringAngles& msg) {
  front_steer = msg.front_angle;
  rear_steer = msg.rear_angle;
}

ros::Subscriber<WAFL2025::WheelSpeeds> wheel_sub("/wheel_speeds", &wheelSpeedsCallback);
ros::Subscriber<WAFL2025::SteeringAngles> steer_sub("/steering_angles", &steeringAnglesCallback);


void steer_stepper(float error)
{
  if(steering_flag==1)
  {
    digitalWrite(FRONT_STEER_DIR, error > 0 ? HIGH : LOW);
    for (int i=0 ; i<200 ; i++)
    {
      digitalWrite(FRONT_STEER_STEP, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(FRONT_STEER_STEP, LOW);
      delayMicroseconds(stepDelay);
    }
  }
}

void setup() {

  Serial.begin(57600); 

  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), encoderISR, CHANGE);

  last_update_time = millis();
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
  pinMode(potPin, INPUT);
  
  nh.getHardware()->setBaud(19200);  // Set baud rate
  nh.initNode();
  nh.advertise(sensor_pub);
  nh.subscribe(wheel_sub);
  nh.subscribe(steer_sub);

  intialpotValue = analogRead(potPin);
  
  float sensor_data_array[4];  // Static array declaration
  sensor_msg.data = sensor_data_array;
  sensor_msg.data_length = 4;
}

void loop() {
  
  unsigned long current_time = millis();

  if (current_time - last_update_time >= UPDATE_INTERVAL) {
    noInterrupts();  // Temporarily disable interrupts to read pulse_count safely
    long pulses = pulse_count;
    pulse_count = 0; // Reset after reading
    int current_direction = direction;
    interrupts();    // Re-enable interrupts

    // Calculate RPM
    float revolutions = (float)pulses / (float)PPR;
    rpm = (80/3)*(revolutions * 600.0) / (float)UPDATE_INTERVAL; // (600 = 60 sec / 0.1 sec)

    // Adjust sign based on direction
    rpm = rpm * current_direction;

    // Print RPM
    Serial.print("RPM: ");
    Serial.println(rpm);

    last_update_time = current_time;
  }

  // Motor control
  digitalWrite(LEFT_MOTOR_DIR, left_speed < 0 ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM, abs(left_speed));
  digitalWrite(RIGHT_MOTOR_DIR, right_speed < 0 ? HIGH : LOW);
  analogWrite(RIGHT_MOTOR_PWM, abs(right_speed));

  // Steering control
  for (int i =0 ; i<50 ; i++)
  {
    potValue = analogRead(potPin) - intialpotValue ;  // 0 to 1023
    //Map potentiometer to angle (0 to 360 degrees)
    currentAngle = (map(potValue, 0, 1023, 0, 3600)/4)*2.5;
    acc +=currentAngle;
  }
  currentAngle=acc/50;
  acc=0;
  float error = (front_steer*180)/3.14 - currentAngle;


  sensor_msg.data[0] = (2*3.14*rpm)/60;
  sensor_msg.data[1] = (2*3.14*rpm)/60;
  sensor_msg.data[2] = 0;
  sensor_msg.data[3] = (currentAngle*3.14)/180;
  sensor_pub.publish(&sensor_msg);

  if(abs(error) > 8)
  {
    steer_stepper(error); 
  }
  else
  {
    error=0;
  }
  
  nh.spinOnce();
  delay(20);  // Adjust delay for desired update rate
}

// Interrupt Service Routine for Encoder
void encoderISR() {
  bool A = digitalRead(ENCODER_PIN_A);
  bool B = digitalRead(ENCODER_PIN_B);

  if (A == B) {
    direction = 1;  // CW
  } else {
    direction = -1; // CCW
  }

  pulse_count++;
}
