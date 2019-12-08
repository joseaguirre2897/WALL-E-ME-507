/** @file Autonomous_WALL_E.cpp
 *    This program makes WALL-E run in Autonomous mode. He move forward until he encounters
 *    a wall 15 inches away. Proceeds to turn his head to the right and left determines
 *    which side is open then turns in that direction.
 *
 *  @author Jose Aguirre
 *  @date 2019-Dec-7 Original file
 *  @copyright (c) 2019 by Jose Aguirre, released under LGPL version 3
 */
#include "Arduino.h"
#include "Wire.h"
#include "PrintStream.h"
#include "task_share.h"                     // From the ME507 Arduino library
#include "task_base.h"
#include <Adafruit_PWMServoDriver.h>

// Servo step up1`
//The setup function is called once at startup of the sketch
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// Which servo channels control which part of the robot
/**
 *  Sets left arm to servo channel 0
 */
const int left_motor =0;
/**
 *  Sets right arm to servo channel 1
 */
const int right_motor = 1;
/**
 *  Sets bottom neck joint to servo channel 2
 */
const int bottom_neck =2;
/**
 *  Sets top neck joint to servo channel 3
 */
const int top_neck = 3;
/**
 *  Sets head to servo channel 4
 */
const int pivot = 4;
/**
 *  Set left eye to servo channel 5
 */
const int left_eye = 5;
/**
 *  Set right eye to servo channel 6
 */
 const int right_eye = 6;
 // setting PWM properties
 /** This sets freq equal to 30000 for the for the motors controller
  */
 const int freq = 30000;
 /** This sets motorAchannel to timer channel 0
  *
  */
 const int motorAchannel = 0;
 /** This sets motorBchannel to timer channel 1
  *
  */
 const int motorBchannel = 1;
 /** This sets resolution equal to 8 out of 16. This allows range from 0 to 255.
  *
  */
 const int resolution = 8;
 /** Defines SERVOMIN as 150. This is before the servo bottoms out.
  */
#define SERVOMIN  200 // This is the 'minimum' pulse length count (out of 4096)
 /** Defines SERVOMAX as 350. This is before the servo maxs out.
  */
#define SERVOMAX  500 // This is the 'maximum' pulse length count (out of 4096)
 /** Defines Servo_FREQ to 60 Hz.
  */
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates


//int wall;

// Motor pins
/** This defines pin 27 as AIN1
 */
const int AIN1_pin = 27;
/** This defines pin 14 as AIN2
 */
const int AIN2_pin = 14;
/** This defines pin 12 as PWM input to motor A
 */
const int PWM_A_pin = 12;
/** This defines pin 15 as BIN1
 */
const int BIN1_pin = 15;
/** This defines pin 32 as BIN2
 */
const int BIN2_pin = 32;
/** This defines pin 13 as PWM input to motor B
 */
const int PWM_B_pin = 13;
/** This defines pin 33 as STBY
 */
const int STBY_pin = 33;
/** Sets the sonar pin to 34
 */
int Sonar_pin = 34;
/** Distance is the average distance of the sonar sensors.
 */
int distance;
/** Boolean to set WALL-E to move forward.
 */
Share <int> forward;
/** Boolean to set WALL-E to turn left
 */
Share <int> Left_turn;
/** Boolean to set WALL-E to turn right
 */
Share <int> Right_turn;
/** Shared variable which hold the distance value to the left of WALL-E
 */
Share <int> Left_distance;
/** Shared variable which hold the distance value to the right of WALL-E
 */
Share <int> Right_distance;
/** Boolean to set when WALL-E is scanning his surroundings
 */
Share <int> scanning;
/** Boolean to set when the sonar finds a WALL-E
 */
Share <int> wall;
/** Boolean to set when WALL-E needs to stop
 */
Share <int> stop;
/** Boolean to set when WALL-E is facing his left side to take sonar data
 */
Share <int> Left;
/** Boolean to set when WALL-E is facing his right side to take sonar data
 */
Share <int> Right;

/** @brief   Master task tells WALL-E if he need to move forward, scan his surroundings,
 * or turn.
 *  @details First the task check if there is a wall in front of WALL-E and WALL-E is not
 *  scanning the area. If there is no wall the WALL-E moves forward. If WALL-E is scanning
 *  WALL-E stops. Then determines if WALL-E need to turn left or right. After WALL-E has
 *  turned scanning boolean is set to zero.
 *  @param parameters Parameters given to task; this task doesn't use any
 */
void Master_Task (void* parameters)
{
	for(;;)
	{
		while (scanning.get()==0 && wall.get() == 0)// && Right.get() == 0 && Left.get() == 0)//(sonar_data.get() > 20)
		{
			Serial.print("No Wall\r\n");
			forward.put(1);
		}
		while(scanning.get() == 1)//(scanning.get() == 1)
		{
			Serial.print("stopping\r\n");
			forward.put(0);
			stop.put(1);

		if((Left_distance.get() > Right_distance.get()))
		{
			Serial.print("LEFT_FLAG\r\n");
			Left_turn.put(1);
			scanning.put(0);
			delay(200);
		}
		else if((Left_distance.get() <= Right_distance.get()))
		{
			Serial.print("RIGHT_FLAG\r\n");
			Right_turn.put(1);
			scanning.put(0);
			delay(200);
		}
		}
		vTaskDelay (1000);
	}
}
/** @brief   The Sonar task determines the average distance from an object
 *  @details First the variable distance is set to zero then the first three readings
 *  are averaged to limit the noise from the sonar sensor. If the average distance is
 *  greater than 15 inches the wall boolean is set to zero. If the average distance is
 *  less than 15 inches wall boolean is set to one. If WALL-E is scanning it stores the
 *  distance values of left and right side.
 *  @param parameters Parameters given to task; this task doesn't use any
 */
void Sonar_Task (void* parameters)
{
	for(;;)
	{
		distance = 0;
		for(int i = 0; i<3;i++)
		{
			distance += (pulseIn(Sonar_pin, HIGH))/147;
		}
		distance = distance/3;
		if (distance > 15 )
		{
			wall.put(0);
		}
		if (distance <= 15)
		{
			wall.put(1);
			scanning.put(1);
		}
		if (scanning.get() ==1 && Left.get() == 1 && Right.get() == 0)
		{
			Left_distance.put((pulseIn(Sonar_pin, HIGH))/147);
			Serial.print("Left distance is");
			Serial.print(Left_distance.get());
			Serial.print("\r\n");
		}
		if (scanning.get() ==1 && Right.get() == 1 && Left.get()==1)
		{
			Right_distance.put((pulseIn(Sonar_pin, HIGH))/147);
			Serial.print("Right distance is");
			Serial.print(Right_distance.get());
			Serial.print("\r\n");
		}
		distance=0;
		vTaskDelay (1000);
	}
}

// This motor task move the motor fowards, backwards, left or right
/** @brief   Motor task controls the two DC motors
 *  @details This task waits to for a boolean to move forwards, stop, left turn,
 *  and right turn.
 *
 *  @param parameters Parameters given to task; this task doesn't use any
 */
void Motor_Task (void* parameters)//, int left,int right)
{
	for(;;)
	{
	if (forward.get() == 1)
	{
		Serial.print("Moving Forward!");
	    ledcWrite(motorAchannel, 255);
	    ledcWrite(motorBchannel, 255);
	    digitalWrite(AIN1_pin, HIGH);
	    digitalWrite(AIN2_pin, LOW);
	    digitalWrite(BIN1_pin, LOW);
	    digitalWrite(BIN2_pin, HIGH);
	}
	if (stop.get() == 1 )//&& Left_turn.get() == 0 && Right_turn.get() == 0)
	{
		Serial.print("STOP!");
	    ledcWrite(motorAchannel, 255);
	    ledcWrite(motorBchannel, 255);
	    digitalWrite(AIN1_pin, LOW);
	    digitalWrite(AIN2_pin, LOW);
	    digitalWrite(BIN1_pin, LOW);
	    digitalWrite(BIN2_pin, LOW);
	}

	// turn left
	if(Left_turn.get() == 1)
	{
		Serial.print("left turn!");
	    ledcWrite(motorBchannel, 255);
	    digitalWrite(AIN1_pin, LOW);
	    digitalWrite(AIN2_pin, HIGH);
	    digitalWrite(BIN1_pin, LOW);
	    digitalWrite(BIN2_pin, HIGH);
	}
	// turn right
	if(Right_turn.get() == 1 && forward.get() ==0)
	{
		Serial.print("right turn!");
	    ledcWrite(motorAchannel, 255);
	    ledcWrite(motorBchannel, 255);
	    digitalWrite(AIN1_pin, HIGH);
	    digitalWrite(AIN2_pin, LOW);
	    digitalWrite(BIN1_pin, HIGH);
	    digitalWrite(BIN2_pin, LOW);
	}
	vTaskDelay (100);
	}
}
/** @brief   Servo task moves WALL-E's head to the left and right directions to
 * then sets the booleans to obtain the distances.
 *  @details If the scanning boolean is one WALL-E's head is turned to the left side and
 *  sets the Left boolean to one. Then turn WALL-E's head to the right and sets the Right
 *  boolean to one.
 *  @param parameter Parameters given to task; this task doesn't use any
 */
void Servo_Task (void* parameter)
{
	for(;;)
	{
		if (scanning.get() == 1)
		{
			Left.put(0);
		    Right.put(0);
			for (uint16_t pulselen = 300; pulselen > 50; pulselen--)
			  {
				pwm.setPWM(pivot, 0, pulselen);
				delay(5);
			  }
			Left.put(1);
			Serial.print("Looking left\r\n");
			delay(100);

			for (uint16_t pulselen = 50; pulselen < 600; pulselen++)
			{
				pwm.setPWM(pivot, 0, pulselen);
				delay(5);
			}

			Serial.print("Looking Right\r\n");
			delay(100);
			Right.put(1);
			for (uint16_t pulselen = 600; pulselen > 300; pulselen--)
			{
				pwm.setPWM(pivot, 0, pulselen);
				delay(5);
			}
		}
		vTaskDelay (1000);
	}
}

/*// The IMU class is used to measure how much the wall has rotated

void IMU_Task (void* parameters)
{

}

*/
/** @brief   Code to start the application.
 *  @details Setup the PWM outputs, I2C lines, and starts the tasks.
 */

void setup()
{
	  ledcSetup(motorAchannel, freq, resolution);
	  ledcSetup(motorBchannel, freq,resolution);
	  // attach the channel to the GPIO to be controlled
	  ledcAttachPin(PWM_A_pin, motorAchannel);
	  ledcAttachPin(PWM_B_pin,motorBchannel);
	  pinMode(AIN1_pin,OUTPUT);
	  pinMode(AIN2_pin,OUTPUT);
	  pinMode(BIN1_pin,OUTPUT);
	  pinMode(BIN2_pin,OUTPUT);
	  pinMode(STBY_pin,OUTPUT);
	  digitalWrite(STBY_pin, HIGH);

	 int SDA_pin = 21;
	 int SCL_pin = 22;
	 Wire.begin(SDA_pin, SCL_pin);
	 Serial.begin (115200);
	 pinMode(Sonar_pin, INPUT);
	  pwm.begin();
	  pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz
	  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~60 Hz updates
	  scanning.put(0);
	  wall.put(0);
	  Right.put(0);
	  Left.put(0);

	 Task Master (Master_Task,        // The task function
			"Master Mind",                       // A name just for humans
			1,                            // Priority, 0 being the lowest
		    1024);                        // Stack size in bytes
	 Task Sonar (Sonar_Task,        // The task function
			"Sonar",                       // A name just for humans
			3,                            // Priority, 0 being the lowest
		    1024);                        // Stack size in bytes
	 Task Motor (Motor_Task,        // The task function
			"Motor",                       // A name just for humans
			2,                            // Priority, 0 being the lowest
		    1024);                        // Stack size in bytes

	 Task Servo (Servo_Task,        // The task function
			"Servo",                       // A name just for humans
			0,                            // Priority, 0 being the lowest
		    1024);                        // Stack size in bytes

/*
	 Task IMU (IMU_Task,        // The task function
			"Master Mind",                       // A name just for humans
			2,                            // Priority, 0 being the lowest
		    1024);                        // Stack size in bytes
		    */
}


// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}
