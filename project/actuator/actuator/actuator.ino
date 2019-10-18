
#include <RedBot.h>  
RedBotMotors motors; 
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
      serialDataMode = receivedChars[0];
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
      newData = false;
    }
}  
