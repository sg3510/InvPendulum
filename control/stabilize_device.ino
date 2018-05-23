
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#include <Wire.h>

// I2Cdev and L3GD20H must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include <I2Cdev.h>
#include <L3GD20H.h>
#include <MPU6050.h>

L3GD20H gyro;
MPU6050 mpu;

// define global vars
int16_t avx, avy, avz; // vars for use for gyro
int16_t accY, accZ, gyroX; // vars for use with mpu
double dvx, dvy, dvz, x, y, z, xBias, yBias, zBias;
double timeStep, timeCurr, timePrev;

volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;

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

// hardware select for MPU / gyro
#define useMPUSensor false
#define isHighPassFilterEnabled false // flag for enabling hardware high pass filter on gyro

void setup() {

  if(useMPUSensor) {
      // run setup routine for MPU6050
      setup_MPU6050();
  } else {
      // run setup routine for L3GH20H
      setup_L3GH20H();
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin1, OUTPUT);
  pinMode(leftMotorDirPin2, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin1, OUTPUT);
  pinMode(rightMotorDirPin2, OUTPUT);

  // calculate min/max PWM output values based on supply voltage and motor tolerence
  minPWMout = 0;  // TODO(nick): implement min voltage cutoff
  maxPWMout = min(255 * voltageMotorMax / voltageSupply, 255);

  // reset current angular positions to 0 and get current timestamp before beginning loop
  x = 0, y = 0, z = 0;
  timeCurr = millis();
}

void setup_L3GH20H() {
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing L3GH20H...");
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

void setup_MPU6050() {

    // join I2C bus (I2Cdev library doesn't do this automatically)
    Wire.begin();

    // initialize serial communication
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing MPU6050...");
    mpu.initialize();

    // set MPU offsets
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
}

void loop() {

    // get updated angular position
    currentAngle = getAngularPosition();

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

float getAngularPosition() {

    float angleValue = 0;

    // get angle of inclination from MPU or gyro sensor depending on hardware selection
    if(useMPUSensor){
      
        // read acceleration and gyroscope values from MPU6050
        accY = mpu.getAccelerationY();
        accZ = mpu.getAccelerationZ();
        gyroX = mpu.getRotationX();
        
        // calculate time delta since last iteration
        timePrev = timeCurr;
        timeCurr = millis();
        timeStep = ((timeCurr - timePrev) / 1000.0);
    
        // calculate the angle of inclination
        accAngle = atan2(accY, accZ) * RAD_TO_DEG;
        gyroRate = map(gyroX, -32768, 32767, -250, 250);
        gyroAngle = (float)gyroRate * timeStep;
    
        // angle of inclination is calculated using a complementary filter which
        // uses data from the gyro and accelerometer in the MPU
        angleValue = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * (accAngle);
        
   } else{
    
        // read raw angular velocity measurements from L3GD20H gyro
        gyro.getAngularVelocity(&avx, &avy, &avz);
    
        // calculate time delta since last iteration
        timePrev = timeCurr;
        timeCurr = millis();
        timeStep = ((timeCurr - timePrev) / 1000.0);
        
        // scale raw velocity measurements using 0.00875F (from L3GD20H datasheet)
        dvx = avx * 0.00875F - xBias;
        dvy = avy * 0.00875F - yBias;
        dvz = avz * 0.00875F - zBias;
    
        // apply thresholding to eliminate sensor drift
        if (abs(dvx) <= 0.5) dvx = 0.0;
        if (abs(dvy) <= 0.5) dvy = 0.0;
        if (abs(dvz) <= 0.5) dvz = 0.0;
    
        // calculate angles in each axis
        x = normaliseAngle(x + (dvx * timeStep));
        y = normaliseAngle(y + (dvy * timeStep));
        z = normaliseAngle(z + (dvz * timeStep));
    
        // dump x,y,z values to serial out for debugging
        Serial.print("Absolute angle (degrees):\t");
        Serial.print(x, DEC); Serial.print("\t");
        Serial.print(y, DEC); Serial.print("\t");
        Serial.print(z, DEC); Serial.print("\t");
        Serial.println();
    
        // Use y axis angle for angle of inclination
        angleValue = y;
    }
       
    // dump angleValue to serial out for debugging
    Serial.print("Angle of inclination (degrees):\t");
    Serial.print(angleValue, DEC);
    Serial.println();

    // return angle of inclination
    return angleValue;
}

double normaliseAngle(double angle)
{
    double newAngle = angle;
    while (newAngle <= -180.0) newAngle += 360.0;
    while (newAngle > 180.0) newAngle -= 360.0;
    return newAngle;
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
