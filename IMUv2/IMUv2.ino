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


void setup() {
    // We initialize the I2C communication
    Wire.begin();

    // We initialize the serial communication
    Serial.begin(115200);

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
    Serial.println("Calibrating the system......... \nPlease drop the device over a flat surface");
    delay(100);
    calibration = calibrateSensor();
    Serial.println("Starting");
    delay(1000);
}

void loop() {
    // Record the start time
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
    

    Serial.print(" Roll: ");
    Serial.print(Roll + calibration[0]);
    Serial.print(" Pitch: ");
    Serial.print(Pitch + calibration[1]);
    Serial.print(" Yaw: ");
    Serial.println(Yaw + calibration[2]);
    

    if(dt == 0.0){
        endTime = micros();
        dt = float(endTime - startTime) / 1000000.0; 
    }

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

    Serial.print(initial_x);
    Serial.print(initial_y);
    Serial.print(initial_z);

    // Calculate the offset (difference between measured and desired temperature)
    float offset_x = 0.0 - initial_x;
    float offset_y = 0.0 - initial_y;
    float offset_z = 0.0 - initial_z;

    offset[0] = offset_x;
    offset[1] = offset_y;
    offset[2] = offset_z;

  // Print calibration information
    Serial.print("Calibration Offset Roll: ");
    Serial.print(offset_x);
    Serial.print(" Calibration Offset Pitch: ");
    Serial.print(offset_y);
    Serial.print(" Calibration Offset Yaw: ");
    Serial.println(offset_z);

    return offset;
}