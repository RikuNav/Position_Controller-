
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>

#include <Wire.h>

#include <cmath>

#define MPU 0x68
#define RES 32768.0
#define CALIB_TIME

unsigned long startTime, endTime;
double dt;
int16_t Acc_x, Acc_y, Acc_z, Gyr_x, Gyr_y, Gyr_z, trash;
float Accx, Accy, Accz, GyrX, GyrY, GyrZ, RollAcc, PitchAcc, RollGyr, PitchGyr, Yaw, Roll, Pitch;
float* calibration;

rcl_publisher_t publisher_roll, publisher_pitch, publisher_yaw;
rcl_subscription_t subscriber_pwm;
std_msgs__msg__Int32 msg, msg_pwm_normalized;
std_msgs__msg__Float32 msg_yaw, msg_pitch, msg_roll;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer1, timer2;

#define LED_PIN 2

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//tiempo de muestreo
#define DT 0.02

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void imu_callback(rcl_timer_t * timer1, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if(timer1 != NULL) {

    //Access the registers of the accelerometer and the gyroscope

    Wire.beginTransmission(MPU);

    //We send the direction of the register we are gonna read
    Wire.write(0x3B);
    Wire.endTransmission(false);

    //We read the data from the accelerometer
    Wire.requestFrom(MPU, 14, false);

    //Obtain the measure of acceleration in x
    Acc_x = (Wire.read() << 8 | Wire.read());

    //Obtain the measure of acceleration in y
    Acc_y = (Wire.read() << 8 | Wire.read());
    
    //Obtain the measure of acceleration in z
    Acc_z = (Wire.read() << 8 | Wire.read());

    //Obtain the temperature
    trash = (Wire.read() << 8 | Wire.read());

    //Obtain the measure of speed in x
    Gyr_x = (Wire.read() << 8 | Wire.read());

    //Obtain the measure of speed in y
    Gyr_y = (Wire.read() << 8 | Wire.read());

    //Obtain the measure of speed in z
    Gyr_z = (Wire.read() << 8 | Wire.read());

    Accx = Acc_x * 2 / RES;
    Accy = Acc_y * 2 / RES;
    Accz = Acc_z * 2 / RES;
    GyrX = Gyr_x * 250 / RES;
    GyrY = Gyr_y * 250 / RES;
    GyrZ = Gyr_z * 250 / RES;

    RollAcc = atan2(Accy, sqrt((pow(Accx, 2)) + (pow(Accz, 2))));
    PitchAcc = atan2(Accx, sqrt((pow(Accy, 2)) + (pow(Accz, 2))));

    RollGyr = Roll + (GyrX * DT);
    PitchGyr = Pitch + (GyrY * DT);
    Yaw = Yaw + (GyrZ * DT);

    RollAcc = (RollAcc * 180) / PI;
    PitchAcc = (PitchAcc * 180) / PI;
    
    Roll = (.95 * RollGyr) + (.05 * RollAcc);
    Pitch = (.95 * PitchGyr) + (.05 * PitchAcc);
    

    msg_roll.data = Roll + calibration[0];
    msg_pitch.data = Pitch + calibration[1];
    msg_yaw.data = Yaw + calibration[2];

    RCSOFTCHECK(rcl_publish(&publisher_roll, &msg_roll, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_pitch, &msg_pitch, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_yaw, &msg_yaw, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "IMU_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_roll,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "roll"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_pitch,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pitch"));

  RCCHECK(rclc_publisher_init_default(
    &publisher_yaw,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "yaw"));

  // create timer,
  const unsigned int timer_1 = 20;
  RCCHECK(rclc_timer_init_default(
    &timer1,
    &support,
    RCL_MS_TO_NS(timer_1),
    imu_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer1));
  
  // We initialize the I2C communication
  Wire.begin();

  //We build the message we are going to send
  
  //First we give the direction of the device
  Wire.beginTransmission(MPU);
  
  //The direction of the register we wanna modify
  Wire.write(0x6B);
  
  //The value we are charging into the register
  Wire.write(0b00000000);
  Wire.endTransmission(true);
  Roll = 0.0;
  Pitch = 0.0;
  Yaw = 0.0;
  dt = 0.0;

  calibration = calibrateSensor();
  delay(1000);

}

void loop() {
  delay(1);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}

// Function to calibrate the sensor and calculate offset
float* calibrateSensor() {
    // Measure the initial temperature (multiple readings for accuracy)
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;
    static float offset[2];
    int numReadings = 500; // Adjust number of readings for accuracy

    for (int i = 0; i < numReadings; i++) {
        if(dt == 0.0){
            startTime = micros();
        }

        //Access the registers of the accelerometer and the gyroscope

        Wire.beginTransmission(MPU);

        //We send the direction of the register we are gonna read
        Wire.write(0x3B);
        Wire.endTransmission(false);

        //We read the data from the accelerometer
        Wire.requestFrom(MPU, 14, false);

        //Obtain the measure of acceleration in x
        Acc_x = (Wire.read() << 8 | Wire.read());

        //Obtain the measure of acceleration in y
        Acc_y = (Wire.read() << 8 | Wire.read());
        
        //Obtain the measure of acceleration in z
        Acc_z = (Wire.read() << 8 | Wire.read());

        //Obtain the temperature
        trash = (Wire.read() << 8 | Wire.read());

        //Obtain the measure of speed in x
        Gyr_x = (Wire.read() << 8 | Wire.read());

        //Obtain the measure of speed in y
        Gyr_y = (Wire.read() << 8 | Wire.read());

        //Obtain the measure of speed in z
        Gyr_z = (Wire.read() << 8 | Wire.read());

        Accx = Acc_x * 2 / RES;
        Accy = Acc_y * 2 / RES;
        Accz = Acc_z * 2 / RES;
        GyrX = Gyr_x * 250 / RES;
        GyrY = Gyr_y * 250 / RES;
        GyrZ = Gyr_z * 250 / RES;

        RollAcc = atan2(Accy, sqrt((pow(Accx, 2)) + (pow(Accz, 2))));
        PitchAcc = atan2(Accx, sqrt((pow(Accy, 2)) + (pow(Accz, 2))));

        RollGyr = Roll + (GyrX * dt);
        PitchGyr = Pitch + (GyrY * dt);
        Yaw = Yaw + (GyrZ * dt);

        RollAcc = (RollAcc * 180) / PI;
        PitchAcc = (PitchAcc * 180) / PI;
        
        Roll = (.95 * RollGyr) + (.05 * RollAcc);
        Pitch = (.95 * PitchGyr) + (.05 * PitchAcc);
        
        sum_x += Roll;
        sum_y += Pitch;
        sum_z += Yaw;
        
        if(dt == 0.0){
            endTime = micros();
            dt = float(endTime - startTime) / 1000000.0; 
        }
    }

    float initial_x = sum_x / numReadings;
    float initial_y = sum_y / numReadings;
    float initial_z = sum_z / numReadings;

    // Calculate the offset (difference between measured and desired temperature)
    float offset_x = 0.0 - initial_x;
    float offset_y = 0.0 - initial_y;
    float offset_z = 0.0 - initial_z;

    offset[0] = offset_x;
    offset[1] = offset_y;
    offset[2] = offset_z;

    return offset;
}