//#include <hcsr04.h>
#include <RedBot.h>
// Sonar
#define TRIG_PIN 12
#define ECHO_PIN 13

// Internal
#define ACTION_SET_SPEED 103      //'g'
#define ACTION_SET_MODE  104      //'h'
#define ACTION_TURN_TRAVEL 105    //'i'
#define ACTION_LANE_SWITCH 106    //'j'
#define ACTION_INTERSECTION 107   //'k'

#define LINE_FOLLOWING   0
#define LISTENING        1

#define LINETHRESHOLD    700
#define SPEED            60

RedBotMotors motors;
RedBotSensor left_outer = RedBotSensor(A0);
RedBotSensor left = RedBotSensor(A3);
RedBotSensor center = RedBotSensor(A1);
RedBotSensor right = RedBotSensor(A7);
RedBotSensor right_outer = RedBotSensor(A6);

RedBotEncoder encoder = RedBotEncoder(A2, 10);

//HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);

int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
float wheelDiam = 2.56;  // diam = 65mm / 25.4 mm/in
float wheelCirc = PI * wheelDiam; // Redbot wheel circumference = pi*D

int leftSpeed;
int rightSpeed;

int serialDataAction;
int serialDataMode;
int serialDataLeft;
int serialDataRight;
int mode;
int busy;

boolean newData = false;
const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
void setup()
{
  Serial.begin(9600);
  busy = 0;
  mode = LISTENING;
  encoder.clearEnc(BOTH);    // clear the encoder count
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

void driveDistance(float distance, int motorPower)
{
  long lCount = 0;
  long rCount = 0;
  float numRev;
  // debug
  Serial.print("driveDistance() ");
  Serial.print(distance);
  Serial.print(" inches at ");
  Serial.print(motorPower);
  Serial.println(" power.");

  numRev = (float) distance / wheelCirc;
  Serial.println(numRev, 3);
  encoder.clearEnc(BOTH);  // clear the encoder count
  motors.drive(motorPower);

  while (rCount < numRev * countsPerRev)
  {
    // while the left encoder is less than the target count -- debug print
    // the encoder values and wait -- this is a holding loop.
    lCount = encoder.getTicks(LEFT);
    rCount = encoder.getTicks(RIGHT);
    Serial.print(lCount);
    Serial.print("\t");
    Serial.print(rCount);
    Serial.print("\t");
    Serial.println(numRev * countsPerRev);
  }
  // now apply "brakes" to stop the motors.
  motors.brake();
}

void loop() {
  recvWithEndMarker();
  long lCount = encoder.getTicks(LEFT);
  long rCount = encoder.getTicks(RIGHT);
  long sonic = 5000;  // Read from ultrasonic
  volatile float lWheelDist = (float)lCount / countsPerRev * wheelCirc;
  volatile float rWheelDist = (float)rCount / countsPerRev * wheelCirc;

  lWheelDist = 5.0;
  rWheelDist = 5.0;
  char arduino_msg[32];
  sprintf(arduino_msg, "%d,%f,%f,%lu\n", busy, lWheelDist, rWheelDist, sonic);
  // Write something to serial
  Serial.print(String(busy) + "," + String(lWheelDist) + "," + String(rWheelDist) + "," + String(sonic) + ",\n");
  if (newData) {
    serialDataAction = receivedChars[0];
    if (serialDataAction == ACTION_SET_SPEED && mode != LINE_FOLLOWING) {
      serialDataLeft = receivedChars[1];
      serialDataRight = receivedChars[2];
      if ((serialDataLeft - 128) > 0) {
        serialDataLeft -= 128;
        serialDataLeft = -serialDataLeft;
      }
      if ((serialDataRight - 128) > 0) {
        serialDataRight -= 128;
        serialDataRight = -serialDataRight;
      }
      motors.leftMotor(-serialDataLeft);
      motors.rightMotor(serialDataRight);
    } else if (serialDataAction == ACTION_TURN_TRAVEL) {
      int serialDataTheta = receivedChars[1];
      int serialDataDir = receivedChars[2];
      int serialDataDist = receivedChars[3];
      int serialDataSpeed = receivedChars[4];
      // Calculate trajectory
      // Travel
      busy = 1;
    } else if (serialDataAction == ACTION_LANE_SWITCH) {

    }
    else if (serialDataAction == ACTION_SET_MODE) {
      serialDataMode = receivedChars[1];
      mode = serialDataMode - '0';
    }
    newData = false;
  }
  if (mode == LINE_FOLLOWING) {
    //LINE FOLLOWING CODE GOES HERE
    if ((((left_outer.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD)) || ((left_outer.read() < LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD)))
        && !((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD)))
    {
      leftSpeed = -SPEED;
      rightSpeed = SPEED;
    }
    // if only left is black -> move to right
    else if ((left_outer.read() > LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD))
    {
      leftSpeed = -SPEED;
      rightSpeed = 0;
    }
    // if only right is black -> move to left
    else if ((right_outer.read() > LINETHRESHOLD) && (left_outer.read() < LINETHRESHOLD))
    {
      leftSpeed = 0;
      rightSpeed = SPEED;
    }
    else
    {
      leftSpeed = 0;
      rightSpeed = 0;
    }
    // if all sensors are on black or up in the air, stop the motors.
    // otherwise, run motors given the control speeds above.
    if ((left.read() > LINETHRESHOLD) && (center.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD) && (left_outer.read() > LINETHRESHOLD) )
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
