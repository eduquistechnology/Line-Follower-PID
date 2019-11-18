#include <QTRSensors.h>

#define Kp 0 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 0 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 215 // max speed of the robot
#define leftMaxSpeed 215 // max speed of the robot
#define rightBaseSpeed 150 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 150  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2
#define delay_timer 2000
#define delay_cal 30
#define rightMotor1 3
#define rightMotor2 4
#define rightMotorPWM 5
#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 11
#define motorPower 8
int lastError = 0;
QTRSensorsRC qtrrc((unsigned char[]) {
  A0, A1, A2, A3, A4, A5, A6, A7
} , NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A7 i.e. digital pins 14-19 in uno

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  for (int i = 0; i < 100; i++) 
	// calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

    /* un-comment this part out for automatic calibration
      if ( i  < 99 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
       turn_right();
      else
       turn_left();
	* The fucntions turn_left() and turn_right() are not configured in the below code. Please use the following and place it before the loop() fucntion.
	
	void turn_left()
	{
	digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
	}
	void turn_right()
	{
	digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
	}


   */
    qtrrc.calibrate();
  delay(delay_cal);
  wait();
  delay(delay_timer); // wait for 2s to position the bot before entering the main loop

  /* comment out for serial printing

    Serial.begin(9600);
    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMinimumOn[i]);
    Serial.print(' ');
    }
    Serial.println();

    for (int i = 0; i < NUM_SENSORS; i++)
    {
    Serial.print(qtrrc.calibratedMaximumOn[i]);
    Serial.print(' ');
    }
    Serial.println();
    Serial.println();
  */
}

void loop()
{
  unsigned int sensors[8];
  int position = qtrrc.readLine(sensors,QTR_EMITTERS_ON,1); // get calibrated readings along with the line position, refer to the QTR Sensors Arduino Library for more details on line position. Considering the line is Black.
  int error = position - 3500;
  int motorSpeed = Kp * error + Kd * (error - lastError);
  lastError = error;
  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  if (rightMotorSpeed > rightMaxSpeed )
	  rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) 
	  leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) 
	  rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) 
	  leftMotorSpeed = 0; // keep the motor speed positive
    digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
    digitalWrite(rightMotor1, HIGH);
    digitalWrite(rightMotor2, LOW);
    analogWrite(rightMotorPWM, rightMotorSpeed);
    digitalWrite(motorPower, HIGH);
    digitalWrite(leftMotor1, HIGH);
    digitalWrite(leftMotor2, LOW);
    analogWrite(leftMotorPWM, leftMotorSpeed);
}

void wait() {
  digitalWrite(motorPower, LOW);
}
