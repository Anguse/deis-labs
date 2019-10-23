
#include <RedBot.h>  

#define ACTION_SET_SPEED h
#define ACTION_SET_MODE  g
#define LINE_FOLLOWING   0

RedBotMotors motors; 
int serialDataAction;
int serialDataMode;
int serialDataLeft;
int serialDataRight;
boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
void setup()
{
  Serial.begin(9600);
  
}
void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
   
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    } 
}
void loop(){

    recvWithEndMarker();
    if(newData){
      serialDataAction = receivedChars[0];
      if(serialDataAction == ACTION_SET_SPEED && mode != LINE_FOLLOWING){
      	serialDataLeft = receivedChars[1];
      	serialDataRight = receivedChars[2];
	if((serialDataLeft - 128) > 0){
	  serialDataLeft -= 128;
	  serialDataLeft = -serialDataLeft;
	}
	if((serialDataRight - 128) > 0){
	  serialDataRight -= 128;
	  serialDataRight = -serialDataRight;        
	}
	motors.leftMotor(-serialDataLeft);
	motors.rightMotor(serialDataRight);
      }else if(serialDataAction == ACTION_SET_MODE){
	serialDataMode = receivedChars[1];
	mode = serialDataMode;
      }
      newData = false;
    }
  if(mode == LINE_FOLLOWING){
       //LINE FOLLOWING CODE GOES HERE
     //custom controls
    // if outside sensors are equal (both white or both black) and not all middle sensors are black -> move straight
    if((((left_outer.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD)) || ((left_outer.read() < LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD))) 
          && !((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD)))
    {
      leftSpeed = -SPEED; 
      rightSpeed = SPEED;
    }
    // if only left is black -> move to right
    else if((left_outer.read() > LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD))
    {
      leftSpeed = -(SPEED);
      rightSpeed = 0;
    }
    // if only right is black -> move to left
    else if((right_outer.read() > LINETHRESHOLD) && (left_outer.read() < LINETHRESHOLD))
    {
      leftSpeed = 0;
      rightSpeed = SPEED;
    }
    else{
      leftSpeed = 0;
      rightSpeed = 0;
    }
    
    // if all sensors are on black or up in the air, stop the motors.
    // otherwise, run motors given the control speeds above.
    if((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD) && (left_outer.read() > LINETHRESHOLD) )
    {
      motors.stop();
    }
    else
    {
      motors.leftMotor(leftSpeed);
      motors.rightMotor(rightSpeed);
    } 

       
   }
}  
