
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3GD20H.h>

#define Kp 50 // 
#define Ki 0 // 
#define Kd 3 // 


// specific I2C address may be passed here
L3GD20H gyro;

int16_t avx, avy, avz, pwmOutput;
double dvx, dvy, dvz, x, y, z, xBias, yBias, zBias;
double timeStep, timeCurr, timePrev, timeTotal = 0;
double cont; 
double errordt = 0;

#define LED_PIN 13 // (Arduino is 13, Teensy is 6)
bool blinkState = false;

#define enA 5 //
#define in1 3 //
#define in2 4


/*
#define enA 6 //
#define in1 4 //
#define in2 5
#define enB 9 //
#define in3 7
#define in4 8 //
 */


#define enB 10 //
#define in3 8
#define in4 9 //

void setup() {
 pwmOutput=0; 
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
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
    for (int i=0; i <=999; i++){
      gyro.getAngularVelocity(&avx, &avy, &avz);
      x = x + avx * 0.00875F;
      y = y + avy * 0.00875F;
      z = z + avz * 0.00875F;
      delay(10);
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


    timePrev = timeCurr;
    timeCurr = millis();
    timeStep = ((timeCurr - timePrev) / 1000.0);
    
    x = normaliseAngle(x + (dvx*timeStep));
    y = normaliseAngle(y + (dvy*timeStep));
    z = normaliseAngle(z + (dvz*timeStep));

   // errordt += x*timeStep;
    timeTotal += timeStep;
    
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
    //Serial.print("Absolute angle (degrees):\t");
    //Serial.print(x,DEC); Serial.print("\t");
    // Serial.print(errordt,DEC); Serial.print("\t");
    //Serial.print(dvx,DEC); Serial.print("\t"); 
    //Serial.print(timeTotal,DEC); Serial.print("\t"); 

    

    
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

//START THE MOTOR CONTROL 

//cont=Kp*x;
cont=Kp*x + Ki*errordt + Kd*dvx;
//Serial.print(cont,DEC); Serial.print("\t"); 

if (cont<0)
{
  pwmOutput=min(map(-cont,0.5,90,50,90),90); 
  analogWrite(enA,pwmOutput); 
  analogWrite(enB,pwmOutput);

  digitalWrite(in3,LOW); 
  digitalWrite(in4,HIGH);
   digitalWrite(in1,LOW); 
  digitalWrite(in2,HIGH); 
} 
else 
  {
  pwmOutput=min(map(cont,0.5,90,50,90),90); 
  analogWrite(enA,pwmOutput);   
  analogWrite(enB,pwmOutput);

  digitalWrite(in3,HIGH); 
  digitalWrite(in4,LOW);
    digitalWrite(in1,HIGH); 
  digitalWrite(in2,LOW);
  //Serial.print("Reverse"); 
  }
  
    
    //Serial.print(pwmOutput);
    //Serial.println(); 

}


double normaliseAngle(double angle)
{
    double newAngle = angle;
    while (newAngle <= -180.0) newAngle += 360.0;
    while (newAngle > 180.0) newAngle -= 360.0;
    return newAngle;
} 
