/** @file WALL_E.cpp
 *    This program makes the ESP32 a soft access point. Using this you can connect to the ESP32 with a computer or phone.
 *    Then typing the IP address in the web address you obtain access to a page with buttons which control WALL-E.
 *
 *  @author Jose Aguirre
 *  @date 2019-Dec-7 Original file
 *  @copyright (c) 2019 by Jose Aguirre, released under LGPL version 3
 */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WiFiClient.h>
//#include <WiFiAP.h>

/** This sets the name for the ESP32 WiFi name (SSID).
 */
const char *ssid = "ESP32-WALL_E";
/** This sets password as 1L8a7n4e! for
 */
const char *password = "1L8a7n4e!";


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


// called this way, it uses the default address 0x40
/** @brief   Arduino repeated code function, usually unused with FreeRTOS.
 *  @details The @c loop() function is called in an endless loop in a non-RTOS
 *      application. In an Arduino/FreeRTOS program it is often unused but can
 *      run low priority code if one really wants it to.
 */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

/** Defines SERVOMIN as 150. This is before the servo bottoms out.
 */
#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
/** Defines SERVOMAX as 350. This is before the servo maxs out.
 */
#define SERVOMAX  350 // This is the 'maximum' pulse length count (out of 4096)
/** Defines Servo_FREQ to 60 Hz.
 */
#define SERVO_FREQ 60 // Analog servos run at ~60 Hz updates

/** Setup web server port number to 80
 *
 */
WiFiServer server(80);

/** @brief   Setup the I2C lines, DC motors, servos, and wifi.
 *  @details The @c setup() function is used to define the SDA line as pin 21 and SCL line as 22. Setup the Oscillator frequency
 *   as 27 MHz for the 16 channel PWM control. Sets up the ESP32 as an closed access point with a name and password.
 */
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("Configuring access point...");

  // Servo setup
  int SDA_pin = 21;
  int SCL_pin = 22;
  Wire.begin(SDA_pin, SCL_pin);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  //wifi setup
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  server.begin();

  // configure Motor PWM functionalities
  ledcSetup(motorAchannel, freq, resolution);
  ledcSetup(motorBchannel, freq,resolution);
  ledcAttachPin(PWM_A_pin, motorAchannel);
  ledcAttachPin(PWM_B_pin,motorBchannel);
  pinMode(AIN1_pin,OUTPUT);
  pinMode(AIN2_pin,OUTPUT);
  pinMode(BIN1_pin,OUTPUT);
  pinMode(BIN2_pin,OUTPUT);
  pinMode(STBY_pin,OUTPUT);
  digitalWrite(STBY_pin, HIGH);

  delay(10);
}


/** @brief   This code uses Wifi to control WALL-E motors.
 *  @details The @c loop() function creates the page for the controls. The program waits for the client to become available
 *  ,determine which button is press if any, and execute the state associated with button.The controls include forward, stop, reverse
 *  , left turn, right turn, arms, eyes and neck.
 */
void loop()
{
	 WiFiClient client = server.available();   // listen for incoming clients

	  if (client) {                             // if you get a client,
	    Serial.println("New Client.");           // print a message out the serial port
	    String currentLine = "";                // make a String to hold incoming data from the client
	    while (client.connected()) {            // loop while the client's connected
	      if (client.available()) {             // if there's bytes to read from the client,
	        char c = client.read();             // read a byte, then
	        Serial.write(c);                    // print it out the serial monitor
	        if (c == '\n') {                    // if the byte is a newline character

	          // if the current line is blank, you got two newline characters in a row.
	          // that's the end of the client HTTP request, so send a response:
	          if (currentLine.length() == 0) {
	            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
	            // and a content-type so the client knows what's coming, then a blank line:
	            client.println("HTTP/1.1 200 OK");
	            client.println("Content-type:text/html");
	            client.println();

	            // the content of the HTTP response follows the header:
	            client.print("Click <a href=\"/forward\">here</a> to move forward.<br>");
	            client.print("Click <a href=\"/stop\">here</a> to stop.<br>");
	            client.print("Click <a href=\"/reverse\">here</a> to move reverse.<br>");
	            client.print("Click <a href=\"/leftturn\">here</a> to turn left.<br>");
	            client.print("Click <a href=\"/rightturn\">here</a> to turn right.<br>");
	            client.print("Click <a href=\"/leftarm\">here</a> to move left arm.<br>");
	            client.print("Click <a href=\"/rightarm\">here</a> to move right arm.<br>");
	            client.print("Click <a href=\"/headup\">here</a> to move head up.<br>");
	            client.print("Click <a href=\"/headdown\">here</a> to move head down.<br>");
	            client.print("Click <a href=\"/righteye\">here</a> to move right eye.<br>");
	            client.print("Click <a href=\"/lefteye\">here</a> to move left eye.<br>");
	            // The HTTP response ends with another blank line:
	            client.println();
	            // break out of the while loop:
	            break;
	          } else {    // if you got a newline, then clear currentLine:
	            currentLine = "";
	          }
	        } else if (c != '\r') {  // if you got anything else but a carriage return character,
	          currentLine += c;      // add it to the end of the currentLine
	        }
	        // Makes WALL-E move forward
	        if (currentLine.endsWith("GET /forward")) {
			    ledcWrite(motorAchannel, 255);
			    ledcWrite(motorBchannel, 255);
			    digitalWrite(AIN1_pin, HIGH);
			    digitalWrite(AIN2_pin, LOW);
			    digitalWrite(BIN1_pin, LOW);
			    digitalWrite(BIN2_pin, HIGH);
			    delay(15);
	        }
	        // Stops WALL-E
	        if (currentLine.endsWith("GET /stop")) {
			    ledcWrite(motorAchannel, 255);
			    ledcWrite(motorBchannel, 255);
			    digitalWrite(AIN1_pin, LOW);
			    digitalWrite(AIN2_pin, LOW);
			    digitalWrite(BIN1_pin, LOW);
			    digitalWrite(BIN2_pin, LOW);
	        }
	        // Makes WALL-E move in reverse
	        if (currentLine.endsWith("GET /reverse"))
	        {
				ledcWrite(motorAchannel, 255);
				ledcWrite(motorBchannel, 255);
				digitalWrite(AIN1_pin, LOW);
				digitalWrite(AIN2_pin, HIGH);
				digitalWrite(BIN1_pin, HIGH);
				digitalWrite(BIN2_pin, LOW);
	        }
	        // Makes WALL-E turn left
	        if (currentLine.endsWith("GET /leftturn"))
	        {
			    ledcWrite(motorBchannel, 255);
			    digitalWrite(AIN1_pin, LOW);
			    digitalWrite(AIN2_pin, HIGH);
			    digitalWrite(BIN1_pin, LOW);
			    digitalWrite(BIN2_pin, HIGH);
			    delay(15);
	        }
	        // Makes WALL-E turn right
	        if (currentLine.endsWith("GET /rightturn"))
	        {
			    ledcWrite(motorAchannel, 255);
			    ledcWrite(motorBchannel, 255);
			    digitalWrite(AIN1_pin, HIGH);
			    digitalWrite(AIN2_pin, LOW);
			    digitalWrite(BIN1_pin, HIGH);
			    digitalWrite(BIN2_pin, LOW);
			    delay(15);
	        }
	        // Makes WALL-E right arm move up and down
	        if (currentLine.endsWith("GET /rightarm"))
	        {
	        	  Serial.println(1);
	        	  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
	        	    pwm.setPWM(1, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  delay(400);
	        	  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
	        	    pwm.setPWM(1, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  delay(400);
	        }
	        // Makes WALL-E left arm move up and down
	        if (currentLine.endsWith("GET /leftarm"))
	        {
	        	  Serial.println(1);
	        	  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
	        	    pwm.setPWM(0, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  delay(400);
	        	  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
	        	    pwm.setPWM(0, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  delay(400);
	        }
	        // Makes WALL-E neck extend upwards
	        if (currentLine.endsWith("GET /headup"))
	        {

	        	Serial.println(1);
	        	  for (uint16_t pulselen = 200; pulselen < 600; pulselen++)
	        	  {
	        	    pwm.setPWM(2, 0, pulselen);
	        	    pwm.setPWM(3, 0, pulselen);
	        	  }
	        	  delay(400);
	        }
	        // Makes WALL-E neck retract
	        if (currentLine.endsWith("GET /headdown")){
	        	  for (uint16_t pulselen = 600; pulselen > 200; pulselen--)
	        	  {
	        		  pwm.setPWM(2, 0, pulselen);
	        		  pwm.setPWM(3, 0, pulselen);
	        	  }
	        	  delay(400);
	        }
	        // Moves WALL-E right eye up and down
	        if (currentLine.endsWith("GET /righteye"))
	        {
	        	Serial.println(1);
	        	  for (uint16_t pulselen = 190; pulselen < 460; pulselen++)
	        	  {
	        	    pwm.setPWM(6, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  for (uint16_t pulselen = 400; pulselen > 190; pulselen--) {
	        	    pwm.setPWM(6, 0, pulselen);
	        	    delay(5);
	        	  }

	        	  delay(400);
	        }
	        // Move WALL-E left eye up and down
	        if (currentLine.endsWith("GET /lefteye"))
	        {
	        	Serial.println(1);
	        	  for (uint16_t pulselen = 500; pulselen > 300; pulselen--) {
	        	    pwm.setPWM(5, 0, pulselen);
	        	    delay(5);
	        	  }
	        	  delay(400);
	        	for (uint16_t pulselen = 300; pulselen < 500; pulselen++)
	        	  {
	        	    pwm.setPWM(5, 0, pulselen);
	        	    delay(5);
	        	  }
	        }
	        }
	      }
	    }
	    // close the connection:
	    client.stop();
	    Serial.println("Client Disconnected.");
	  }
