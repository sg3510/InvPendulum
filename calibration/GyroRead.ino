
// I2C device class (I2Cdev) demonstration Arduino sketch for L3GD20H class
// 3/05/2015 by Nate Costello <natecostello at gmail dot com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2015-03-05 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jonathan Arnett, Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3GD20H.h>



// specific I2C address may be passed here
L3GD20H gyro;

int16_t avx, avy, avz;
double dvx, dvy, dvz, x, y, z, xBias, yBias, zBias;
double timeStep, timeCurr, timePrev;


#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    gyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(gyro.testConnection() ? "L3GD20H connection successful" : "L3GD20H connection failed");

    //test endian functions
    gyro.setEndianMode(not gyro.getEndianMode());
    Serial.print("EndianMode: ");
    Serial.print(gyro.getEndianMode());
    gyro.setEndianMode(not gyro.getEndianMode());
    Serial.print(" EndianMode: ");
    Serial.println(gyro.getEndianMode());

    //Get ODR
    gyro.setOutputDataRate(100);
    Serial.print("Data Rate:");
    Serial.println(gyro.getOutputDataRate());

    

    //Enable High Pass Filter
    //Serial.println("Enabling High Pass Filter");
    //gyro.setDataFilter(L3GD20H_HIGH_PASS);
    //gyro.setHighPassFilterEnabled(true);
    //gyro.setHighPassMode(L3GD20H_HPM_NORMAL);
    //gyro.setHighPassFilterReference(L3GD20H_HPCF10);
    //gyro.setHighPassFilterCutOffFrequencyLevel(L3GD20H_HPCF10);
    

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    // set scale to 250
    gyro.setFullScale(250);
    
    gyro.getAngularVelocity(&avx, &avy, &avz);
    delay(200);
    for (int i=0; i <= 999; i++){
      gyro.getAngularVelocity(&avx, &avy, &avz);
      x = x + avx * 0.00875F;
      y = y + avy * 0.00875F;
      z = z + avz * 0.00875F;
      delay(5);
    }
    xBias = x / 1000.0;
    yBias = y / 1000.0;
    zBias = z / 1000.0;
    x =0 ,y = 0,z = 0;
    timeCurr = millis();
    
}

void loop() {
    // read raw angular velocity measurements from device
    gyro.getAngularVelocity(&avx, &avy, &avz);
    // int16_t x_single = gyro.getAngularVelocityX();
    // int16_t y_single = gyro.getAngularVelocityY();
    dvx = avx * 0.00875F - xBias;
    dvy = avy *0.00875F - yBias;
    dvz = avz *0.00875F - zBias;

    if (abs(dvx) <= 0.5) dvx = 0.0;
    if (abs(dvy) <= 0.5) dvy = 0.0;
    if (abs(dvz) <= 0.5) dvz = 0.0;
 

    timePrev = timeCurr;
    timeCurr = millis();
    timeStep = ((timeCurr - timePrev) / 1000.0);
    
    x = normaliseAngle(x + (dvx*timeStep));
    y = normaliseAngle(y + (dvy*timeStep));
    z = normaliseAngle(z + (dvz*timeStep));
    
    // //read X memory address directly
    // uint8_t xval_l, xval_h;
    // uint8_t devAddress = 0x6B;
    // uint8_t regAddXL = 0x28;
    // uint8_t regAddXH = 0x29;
    // I2Cdev::readByte(devAddress, regAddXL, &xval_l);
    // I2Cdev::readByte(devAddress, regAddXH, &xval_h);
    // //read X memory addresses in single sequential
    // uint8_t data[2];
    // I2Cdev::readBytes(devAddress, regAddXL| 0x80, 2, data);
    

    // Serial.print("Direct Mem Read: Xl: "); 
    // Serial.print(xval_l); Serial.print("\tXh: ");
    // Serial.print(xval_h); Serial.print("\t");
    // Serial.print("Direct readBytes data[0] data[1]: ");
    // Serial.print(data[0]); Serial.print("\t");
    // Serial.print(data[1]); Serial.print("\t");
    // Serial.print("Bitshifted: "); Serial.print((((int16_t)data[1]) << 8) | data[0]);
//    Serial.print("angular velocity (dps):\t");
//    Serial.print(dvx,DEC); Serial.print("\t");
//    Serial.print(dvy,DEC); Serial.print("\t");
//    Serial.print(dvz,DEC); Serial.print("\t");
    // Serial.print(" x read only: "); Serial.print(x_single);
    // Serial.print(" y read only: "); Serial.println(y_single);
    Serial.print("Absolute angle (degrees):\t");
    Serial.print(x,DEC); Serial.print("\t");
    Serial.print(y,DEC); Serial.print("\t");
    Serial.print(z,DEC); Serial.print("\t");
    
    Serial.println();
    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    

}


double normaliseAngle(double angle)
{
    double newAngle = angle;
    while (newAngle <= -180.0) newAngle += 360.0;
    while (newAngle > 180.0) newAngle -= 360.0;
    return newAngle;
}


