/*/////////////////////////////////////////////////////////////\
 * 
 * PID Control for the Self Balancing Bot Project
 * Developed by Ayush Jamdar for iBot, CFI IIT Madras
 * To be burnt on the Espressif ESP32 DevBoard 2.0 by Elec Club
 * For Mega Session 2.0, April 2022
 * Special thanks to Arun Palaniappan from Elec Club and all 
 * those who created the libraries.
 * 
 * ////////////////////////////////////////////////////////////
 */


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Motor A
int motor1Pin1 = 16;
int motor1Pin2 = 17;
int enable1Pin = 14;

// Motor B
int motor2Pin1 = 26;
int motor2Pin2 = 27;
int enable2Pin = 25;

// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// resolution is 8, 2^8 = 256. So duty cycle should take values from 0 to 255. Map it accordingly to percentage.
int dutyCycle = 200;

// PID constants
double kp = 025;
double ki = 0;
double kd = 100;
float gyroAngle = 0;
unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError, rateError;

int niter =0;
float  yOffset = -0.0891;

Adafruit_MPU6050 mpu;

void moveForward(int speedDuty)
{ // speedDuty is an int from 0 to 255

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  ledcWrite(pwmChannel, speedDuty);
  Serial.print("Forward with duty cycle: ");
  Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction

  delay(100);
}

void moveBackward(int speedDuty)
{

  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  ledcWrite(pwmChannel, speedDuty);
  Serial.print("Backward with duty cycle: ");
  Serial.println(speedDuty);
  // should maintain this signal until it starts rotating in the opposite direction

  delay(100);
}



void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    delay(10);
  // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin(0x68))
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  sensors_event_t a1, g1, temp1;
  mpu.getEvent(&a1, &g1, &temp1);

  float initial = g1.gyro.x;
  Serial.print("Initial: ");
  Serial.println(initial);
  Serial.println("");
  delay(100);

  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enable1Pin, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED Control PWM functionalities -- setting up a PWM channel to create the signal (there are 16 PWM Channels from 0 to 15)
  // LEDC is used to generate PWM signals, but not necessarily only for LEDs
  ledcSetup(pwmChannel, freq, resolution);

  // attach the channel to the GPIO to be controlled -- channeling the same PWM signal through enable1Pin and enable2Pin
  ledcAttachPin(enable1Pin, pwmChannel);
  ledcAttachPin(enable2Pin, pwmChannel);

  /*
  // testing
  Serial.print("Testing DC Motor...");

  // FIRST TEST BACKWARD AND FORWARD PWM MOTION OF MOTORS
  // setting both motors to 'forward' configuration
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  for (int j = 0; j < 1; j++)
  {
    for (int i = 200; i < 255; i += 5)
    {
      ledcWrite(pwmChannel, i);
      Serial.print("Forward with duty cycle: ");
      Serial.println(i);
      delay(1000);
    }
  }

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  for (int j = 0; j < 1; j++)
  {
    for (int i = 200; i < 255; i += 5)
    {
      ledcWrite(pwmChannel, i);
      Serial.print("Backward with duty cycle: ");
      Serial.println(i);
      delay(1000);
    }
  }

  ledcWrite(pwmChannel, 0);

  // DONE TESTING
  */
}

void loop()
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  //Final zero rate offset in radians/s: 
//-0.0889, 0.0926, -0.0167
  float gyroY = g.gyro.y;
  gyroY += yOffset;
  // PRINT THE ROTATION SPEED OF Y
  Serial.print("Rotation Y: ");
  Serial.print(gyroY);
  Serial.println("");

  // NOW NEED MAKE IT SELF-BALANCE

  int speed; // duty cycle 0 to 255 for motor

  // PID Control Algorithm

  // Calculate angle of inclination
  currentTime = millis();                             // get current time
  elapsedTime = (double)(currentTime - previousTime); // compute time elapsed from previous computation
  float gyroRate = gyroY;
  
  //float gyroRate = map(gyroX, -32768, 32767, -250, 250);    // check these numbers
  gyroAngle += (float)gyroRate * elapsedTime / 1000;  //elapsed time is in milliseconds

  error = gyroAngle - 0;                         // determine error
  cumError += error * elapsedTime;               // compute integral
  rateError = (error - lastError) / elapsedTime; // compute derivative
  //[-174, -173] --> [0,7]

  float out = kp * error + ki * cumError + kd * rateError; // PID output
  Serial.print("gyroangle: ");
  Serial.println(error);
  Serial.print("Out: ");
  Serial.println(out);
  lastError = error; // remember current error
  int base_speed = 200;
  if (out < 0)
  {
    // -ve error means + y axis will tilt downwards
    speed = base_speed + (int)(-out);
    moveForward(speed); // along + y
  }
  else
  {
    speed = base_speed + out;
    moveBackward(speed); // along - y
  }

  previousTime = currentTime;

  
}
