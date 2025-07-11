#include <mpu6050.h>

#define MPU_ADDRESS 0x68 //  mpu6050 address is 0x69 if AD0 pin is powered -  otherwise it's 0x68

float gX, gY, gZ; // initialise gyroscope variables
float aX, aY, aZ; // initialise accelerometer variables
float temp; // initialise temperature variables

void setup(){
    Serial.begin(115200); // begin serial communication at 115200 baud
    wakeSensor(MPU_ADDRESS); // wakes sensor from sleep mode
    Serial.println("t/gx/gy/gz/ax/ay/az/temp");
}

void loop(){
    readGyroData(MPU_ADDRESS , gX, gY, gZ); // pass MPU6050 address and gyroscope values are written to 3 provided variables
    readAccelData(MPU_ADDRESS, aX, aY, aZ); // pass MPU6050 address and accelerometer values are written to 3 provided variables
    readTempData(MPU_ADDRESS, temp); // pass MPU6050 address and temperature values are written to 3 provided variables
    Serial.print(millis());
    Serial.print("/");
    Serial.print(gX);
    Serial.print("/");
    Serial.print(gY);
    Serial.print("/");
    Serial.print(gZ);
    Serial.print("/");
    Serial.print(aX);
    Serial.print("/");
    Serial.print(aY);
    Serial.print("/");
    Serial.print(aZ);
    Serial.print("/");
    Serial.println(temp);
    delay(10);
}