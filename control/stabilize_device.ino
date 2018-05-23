
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3GD20H.h>


// specific I2C address may be passed here
L3GD20H gyro;

// define global vars
int16_t avx, avy, avz;
double dvx, dvy, dvz, x, y, z, xBias, yBias, zBias;
double timeStep, timeCurr, timePrev;

volatile int motorPower;
volatile float currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;

bool blinkState = false;

#define targetAngle 0

// parameters for PID controller
#define Kp  10
#define Ki  0.1
#define Kd  0.001

// arduino output pin configuration
#define leftMotorPWMPin   10
#define leftMotorDirPin1  8
#define leftMotorDirPin2  9
#define rightMotorPWMPin  5
#define rightMotorDirPin1 3
#define rightMotorDirPin2 4
#define LED_PIN 13

// motor supply voltage and min/max operating voltage
#define voltageSupply    9.0
#define voltageMotorMin  0.0
#define voltageMotorMax  5.0
int minPWMout, maxPWMout;

void setup() {
  setup_L3GH20H();

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // calculate min/max PWM output values based on supply voltage and motor tolerence
  minPWMout = 0;  // TODO(nick): implement min voltage cutoff
  maxPWMout = min(255 * voltageMotorMax / voltageSupply, 255);

  // reset current angular positions to 0 and get current timestamp before beginning loop
  x = 0, y = 0, z = 0;
  timeCurr = millis();
}

void setup_L3GH20H() {

    #define isHighPassFilterEnabled false // flag for enabling hardware high pass filter
  
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

    // test endian functions
    gyro.setEndianMode(not gyro.getEndianMode());
    Serial.print("EndianMode: ");
    Serial.print(gyro.getEndianMode());
    gyro.setEndianMode(not gyro.getEndianMode());
    Serial.print(" EndianMode: ");
    Serial.println(gyro.getEndianMode());

    // get output data rate
    gyro.setOutputDataRate(100);
    Serial.print("Data Rate:");
    Serial.println(gyro.getOutputDataRate());

    // use hardware high pass fiter if enabled
    if (isHighPassFilterEnabled) {
      Serial.println("Enabling High Pass Filter");
      gyro.setDataFilter(L3GD20H_HIGH_PASS);
      gyro.setHighPassFilterEnabled(true);
      gyro.setHighPassMode(L3GD20H_HPM_NORMAL);
      gyro.setHighPassFilterReference(L3GD20H_HPCF10);
      gyro.setHighPassFilterCutOffFrequencyLevel(L3GD20H_HPCF10);
    }

    // set scale to 250
    gyro.setFullScale(250);

    // calculate gyro offsets
    gyro.getAngularVelocity(&avx, &avy, &avz);
    delay(200);
    for (int i = 0; i <= 999; i++){
      gyro.getAngularVelocity(&avx, &avy, &avz);
      x = x + avx * 0.00875F;
      y = y + avy * 0.00875F;
      z = z + avz * 0.00875F;
      delay(5);
    }

    // set bias constants for gyro starting position
    xBias = x / 1000.0;
    yBias = y / 1000.0;
    zBias = z / 1000.0;
}

void loop() {

    // calculate angular position using L3GD20H gyro
    // TODO(nick): can I return an array of the three values instead of modifying global vars?
    getAngularPosition_L3GD20H();

    // dump x,y,z values to serial out for debugging
    Serial.print("Absolute angle (degrees):\t");
    Serial.print(x,DEC); Serial.print("\t");
    Serial.print(y,DEC); Serial.print("\t");
    Serial.print(z,DEC); Serial.print("\t");
    Serial.println();

    // Use y axis angle for stabilization basis
    currentAngle = y;

    // calculate error and constrain cumulative errorSum to prevent overflow
    error = currentAngle - targetAngle;
    errorSum = errorSum + error;
    errorSum = constrain(errorSum, -500, 500);
    
    // calculate output from P, I and D values
    motorPower = Kp * (error) + Ki * (errorSum) * timeStep - Kd * (currentAngle - prevAngle) / timeStep;
    prevAngle = currentAngle;

    // set motor power after constraining it to max PWM values
    motorPower = constrain(motorPower, -1 * maxPWMout, maxPWMout);
    setMotors(motorPower, motorPower);

    // dump error values to serial out for debugging
    Serial.print(error, DEC); Serial.print("\t");
    Serial.print(errorSum, DEC); Serial.print("\t");
    Serial.print(motorPower,DEC); Serial.print("\t");
    
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

void getAngularPosition_L3GD20H() {
  
    // read raw angular velocity measurements from L3GD20H gyro
    gyro.getAngularVelocity(&avx, &avy, &avz);

    // scale raw velocity measurements using 0.00875F (from L3GD20H datasheet)
    dvx = avx * 0.00875F - xBias;
    dvy = avy * 0.00875F - yBias;
    dvz = avz * 0.00875F - zBias;

    // apply thresholding to eliminate sensor drift
    if (abs(dvx) <= 0.5) dvx = 0.0;
    if (abs(dvy) <= 0.5) dvy = 0.0;
    if (abs(dvz) <= 0.5) dvz = 0.0;
 
    timePrev = timeCurr;
    timeCurr = millis();
    timeStep = ((timeCurr - timePrev) / 1000.0);
    
    x = normaliseAngle(x + (dvx*timeStep));
    y = normaliseAngle(y + (dvy*timeStep));
    z = normaliseAngle(z + (dvz*timeStep));
    
    // // read X memory address directly
    // uint8_t xval_l, xval_h;
    // uint8_t devAddress = 0x6B;
    // uint8_t regAddXL = 0x28;
    // uint8_t regAddXH = 0x29;
    // I2Cdev::readByte(devAddress, regAddXL, &xval_l);
    // I2Cdev::readByte(devAddress, regAddXH, &xval_h);
    // // read X memory addresses in single sequential
    // uint8_t data[2];
    // I2Cdev::readBytes(devAddress, regAddXL| 0x80, 2, data);
    
    // Serial.print("Direct Mem Read: Xl: "); 
    // Serial.print(xval_l); Serial.print("\tXh: ");
    // Serial.print(xval_h); Serial.print("\t");
    // Serial.print("Direct readBytes data[0] data[1]: ");
    // Serial.print(data[0]); Serial.print("\t");
    // Serial.print(data[1]); Serial.print("\t");
    // Serial.print("Bitshifted: "); Serial.print((((int16_t)data[1]) << 8) | data[0]);
    // Serial.print("angular velocity (dps):\t");
    // Serial.print(dvx,DEC); Serial.print("\t");
    // Serial.print(dvy,DEC); Serial.print("\t");
    // Serial.print(dvz,DEC); Serial.print("\t");
    // Serial.print(" x read only: "); Serial.print(x_single);
    // Serial.print(" y read only: "); Serial.println(y_single);
}


void setMotors(int leftMotorSpeed, int rightMotorSpeed) {

  // set left motor speed, reverse motor direction if specified speed is negative
  if (leftMotorSpeed >= 0) {
    digitalWrite(leftMotorDirPin1, LOW);
    digitalWrite(leftMotorDirPin2, HIGH);
    
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
  }
  else {
    digitalWrite(leftMotorDirPin1, HIGH);
    digitalWrite(leftMotorDirPin2, LOW);
    
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
  }

  // set right motor speed, reverse motor direction if specified speed is negative
  if (rightMotorSpeed >= 0) {
    digitalWrite(rightMotorDirPin1, LOW);
    digitalWrite(rightMotorDirPin2, HIGH);
    
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
  }
  else {
    digitalWrite(rightMotorDirPin1, HIGH);
    digitalWrite(rightMotorDirPin2, LOW);
    
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
  }
}
