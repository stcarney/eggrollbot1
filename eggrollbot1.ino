/* Written by: D. H. Goldthwaite edited by Chris Scianna 9/17/16
 * Revised to be sumo for service-learning by S. Freeman
 * Date:      April, 2017
 * Purpose:    Robot control with Sparkfun RedBoard
 * Modified:   3/23/16 - New and updated redboard-robo-code: CStone_basic_red_robot_drive_code.cpp
 *              9/13/16 - Minor tweaks.
 *              4/25/17 - took out lights
 *              4/25/17 - rewrote description, added action for both front sensors HIGH
 *                        tested with Red Ruckus robot (Don's sumo robot!) - dhg
 * Function    This program implements a very basic algorithm for a sumo competition: using the
 *             side sensors to detect the edge of the playing area and turning to correct course.
 *             The front sensor is also implemented and the code can be adjusted so the robot 
 *             executes whatever action the user desires.
         
         If the right sensor goes HIGH, the right side of the robot is crossing the play 
         area boundary. To correct, the right motor continues to turn forward and the 
         left motor turns backwards (CCW) for a short time, swinging the front of the 
         robot to the left.  If the left sensor goes HIGH, a similar sequence follows, in the
         opposite direction.  

         If a front obstacle is encountered or both side sensors go into the white ring,
         the robot backs up, spins left, goes forward.

         The program includes a serial output indicating when the left or right sensor is active.  
         By tethering the robot to a computer, this feature can be used for calibrating the side 
         sensors.


******************************************************************************************************/
#include <Servo.h>  // servo library

Servo servo_r;                 // Declare right and left servos
Servo servo_l;
const int leftSensor = 11;     // Assign sensor pins 
const int rightSensor = 7;
const int frontSensor = A0;    //Front sensor is analog
int lsensorState = 0;          // Initialize sensor variable
int rsensorState = 0;
float fsensorVoltage, fsensorValue;
const int right_forward = 180;
const int right_backward = 0;
const int left_forward = 0;
const int left_backward = 180;
const int stop_wheel = 90;

void setup(){
  pinMode(leftSensor, INPUT);   // Declare sensor pins as input
  pinMode(rightSensor, INPUT);
  pinMode(frontSensor, INPUT);
  servo_r.attach(10);            // setup servos
  servo_l.attach(11); 
  Serial.begin(9600);           // Output sensor state for testing
}
void loop(){   // Here's where the rubber meets the road
  
  servo_r.write(right_forward);
  servo_l.write(left_forward); 
  rsensorState = digitalRead(rightSensor);  //read right sensor
  lsensorState = digitalRead(leftSensor);   //read left sensora
  fsensorValue = analogRead(frontSensor);
  fsensorVoltage = fsensorValue*(5.0/1023.0);
         
  Serial.print("Front: " + String(fsensorVoltage) + "  Left: " + String(rsensorState) + "  Right: ");
  Serial.println(lsensorState);
         
  
  if(fsensorVoltage > 2.5){
    Serial.println("Front Sensor exceeding Threshold");
    servo_r.write(stop_wheel);        // Back up
    servo_l.write(stop_wheel);
    delay(1000);
    servo_r.write(right_forward);      //Spin Left
    servo_l.write(left_backward);
    delay(750);
    servo_l.write(left_forward);    // Full Speed Ahead
    servo_r.write(right_forward);  
  }
  
  else if(lsensorState == HIGH && rsensorState == LOW){
    
    servo_l.write(left_forward);    // spin to right
    servo_r.write(right_backward);
    Serial.println("Left Sensor");
    delay(1000);
    servo_l.write(left_forward);    // Full Speed Ahead
    servo_r.write(right_forward);
  }
   else if(rsensorState == HIGH && lsensorState == LOW){
    servo_l.write(left_backward);    // spin to left
    servo_r.write(right_forward);
    Serial.println("Right Sensor");
    delay(1000);
    servo_l.write(left_forward);    // Full Speed Ahead
    servo_r.write(right_forward);
  }
   else if(rsensorState == HIGH && lsensorState == HIGH){
    servo_l.write(left_backward);    // spin to left
    servo_r.write(right_backward);
    //Serial.println("Left Sensor & Right Sensor");
    delay(750);
    servo_l.write(left_backward);    // spin to left
    servo_r.write(right_forward);
    delay(1000);
    servo_l.write(left_forward);    // Full Speed Ahead
    servo_r.write(right_forward);
  }
}

void count()
{
  delay (1000);
}
