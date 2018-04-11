/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low gyro data registers.
They can be converted to units of dps (degrees per second) using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An L3GD20H gives a gyro X axis reading of 345 with its
default full scale setting of +/- 245 dps. The So specification
in the L3GD20H datasheet (page 10) states a conversion factor of 8.75
mdps/LSB (least significant bit) at this FS setting, so the raw
reading of 345 corresponds to 345 * 8.75 = 3020 mdps = 3.02 dps.
*/
#define ALPHA 0
#define SCALING 0.00875

#include <Wire.h>
#include <L3G.h>

L3G gyro;


int buffersize = 1000; 
int gyro_deadzone = 1;

float x = 0;
float y = 0;
float z = 0;

float buff_x = 0;
float buff_y = 0;
float buff_z = 0;

float offset_x = 0;
float offset_y = 0;
float offset_z = 0;

void setup() {
  int i;
  Serial.begin(9600);
  Wire.begin();

  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();
  
  /* for loop execution */
  for (i = 0; i < 100; i++) {
    Serial.print("Discarding value #");
    Serial.println(i);
    gyro.read();
    delay(10);
  }
Serial.println("____________________");
  Serial.println("Starting calibration");
  Serial.println("____________________");
  for (i = 0; i < buffersize; i++) {
    gyro.read();
    buff_x += gyro.g.x * SCALING;
    buff_y += gyro.g.y * SCALING;
    buff_z += gyro.g.z * SCALING;
    Serial.print("Iteration #");
    Serial.println(i);
    delay(10);
  }
  offset_x = buff_x/i;
  offset_y = buff_y/i;
  offset_z = buff_z/i;
  Serial.println("Calibration finished:");
  Serial.print("X: ");
  Serial.print(offset_x);
  Serial.print(" Y: ");
  Serial.print(offset_y);
  Serial.print(" Z: ");
  Serial.println(offset_z);
}

void loop() {
  gyro.read();

  x = (float) (gyro.g.x*SCALING - offset_x) * (1.0 - ALPHA) + ALPHA * x;
  y = (float) (gyro.g.y*SCALING - offset_y) * (1.0 - ALPHA) + ALPHA * y;
  z = (float) (gyro.g.z*SCALING - offset_z) * (1.0 - ALPHA) + ALPHA * z;

  Serial.print("G ");
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);

  delay(10);
}
