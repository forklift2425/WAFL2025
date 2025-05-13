#include <Wire.h>
#include <MPU9250_WE.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>

#define MPU9250_ADDR 0x68

// Minimal ROS node configuration
ros::NodeHandle_<ArduinoHardware, 1, 1, 50, 50> nh;

// IMU message
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu/data", &imu_msg);

MPU9250_WE imu(MPU9250_ADDR);

void setup() {
  // Initialize ROS with minimal settings
  nh.getHardware()->setBaud(9600); // Reduced baud rate
  nh.initNode();
  nh.advertise(imu_pub);

  // Initialize IMU with basic settings
  Wire.begin();
  if(!imu.init()) while(1); // Halt if initialization fails
  
  imu.setAccRange(MPU9250_ACC_RANGE_8G);
  imu.setGyrRange(MPU9250_GYRO_RANGE_500);
  imu.setSampleRateDivider(5);
}

void loop() {
  static uint32_t last_time = 0;
  if(millis() - last_time < 20) return; // ~50Hz update rate
  last_time = millis();

  // Read raw sensor data
  xyzFloat accel = imu.getAccRawValues();
  xyzFloat gyro = imu.getGyrRawValues();

  // Populate IMU message (minimal fields only)
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu_link";

  // Linear acceleration (convert g to m/s²)
  imu_msg.linear_acceleration.x = accel.x * 9.80665;
  imu_msg.linear_acceleration.y = accel.y * 9.80665;
  imu_msg.linear_acceleration.z = accel.z * 9.80665;

  // Angular velocity (convert °/s to rad/s)
  imu_msg.angular_velocity.x = gyro.x * (PI/180.0);
  imu_msg.angular_velocity.y = gyro.y * (PI/180.0);
  imu_msg.angular_velocity.z = gyro.z * (PI/180.0);

  // Set orientation to identity (no filtering)
  imu_msg.orientation.w = 1.0;
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;

  // Publish and spin
  imu_pub.publish(&imu_msg);
  nh.spinOnce();
}
