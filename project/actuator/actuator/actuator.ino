
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
   if(mode == LINE_FOLLOLWING){
       //LINE FOLLOWING CODE GOES HERE
   }
}  
