#include <RedBot.h>
#include <TimerOne.h>

// Sonar
#define TRIG_PIN 11
#define ECHO_PIN A5

// Internal
#define ACTION_SET_SPEED 103      //'g'
#define ACTION_SET_MODE  104      //'h'
#define ACTION_TURN_TRAVEL 105    //'i'
#define ACTION_LANE_SWITCH 100    //'d'
#define ACTION_INTERSECTION 107   //'k'

//modes
#define LINE_FOLLOWING   0
#define LISTENING        1

#define LINETHRESHOLD    700

// IOs
RedBotMotors motors;
RedBotSensor left_outer = RedBotSensor(A0);
RedBotSensor left = RedBotSensor(A3);
RedBotSensor right = RedBotSensor(A7);
RedBotSensor right_outer = RedBotSensor(A6);
RedBotEncoder encoder = RedBotEncoder(A2, A4);

//variables for movement
int leftSpeed;
int rightSpeed;
int SPEED;

//serial read variables
int serialDataAction;
int serialDataMode;
int serialDataLeft;
int serialDataRight;
int serialData;
int mode;
int busy;
boolean newData = false;
const byte numChars = 32;
//char receivedChars[numChars];   // an array to store the received data
int receivedChars[numChars];

//movement calculation
int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev
float wheelDiam = 6.5;   // 6.5cm diameter of wheel
float wheelCirc = PI * wheelDiam; // Redbot wheel circumference = pi*D
float rotangleintersec = 70;
float rotdistintersec;

//reading distance
long duration = 0;
long distance = 0;

long lCount;
long rCount;


void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(50);
  mode = LISTENING;
  busy = 0;
  encoder.clearEnc(BOTH);
  rotdistintersec = (rotangleintersec / 360) * PI * 2 * 20;
  SPEED = 60;
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Timer1.initialize(50000); //µs
  Timer1.attachInterrupt(interruptfunc);
}

void interruptfunc() {
  readdist();
  sendvals();
}

/*void recvWithEndMarker() {
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
  }*/

void recString() {
  String rec;
  int i = 0;
  int previdx = 0;
  char help[10];

  if (Serial.available() > 0 && newData == false) {
    rec = Serial.readString();
    newData = true;
  }
  if (newData) {
    int idx = rec.indexOf(',', previdx);
    while (idx != -1) {
      receivedChars[i] = rec.substring(previdx, idx).toFloat();
      if ((receivedChars[i]) == 0 && !(rec.substring(previdx, idx).equals("0"))) {
        rec.substring(previdx, idx).toCharArray(help, 10);
        receivedChars[i] = (float)(help[0]);
      }
      previdx = idx + 1;
      idx = rec.indexOf(',', previdx);
      i++;
    }
    idx = rec.indexOf('\n', previdx);
    receivedChars[i] = rec.substring(previdx, idx).toFloat();
    if ((receivedChars[i]) == 0 && !(rec.substring(previdx, idx).equals("0"))) {
      rec.substring(previdx, idx).toCharArray(help, 10);
      receivedChars[i] = (float)(help[0]);
    }
  }
}

void loop() {
  //recvWithEndMarker();
  recString();
  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);
  if (newData) {
    serialDataAction = (int)receivedChars[0];
    if (serialDataAction == ACTION_SET_SPEED && mode != LINE_FOLLOWING) {
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "," + (String)receivedChars[2] + "\n");
      serialDataLeft = receivedChars[1];
      serialDataRight = receivedChars[2];
  
      motors.leftMotor(-serialDataLeft);
      motors.rightMotor(serialDataRight);
    } else if (serialDataAction == ACTION_SET_SPEED) {
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "\n");
      serialData = receivedChars[1];
     
      SPEED = serialData;
    } else if (serialDataAction == ACTION_TURN_TRAVEL) {
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "," + (String)receivedChars[2] + "," + (String)receivedChars[3] + "," + (String)receivedChars[4] + "\n");
      busy = 1;
      int serialDataTheta = receivedChars[1];
      int serialDataDir = receivedChars[2];
      int serialDataDist = receivedChars[3];
      int serialDataSpeed = receivedChars[4];
      int angle = serialDataTheta - serialDataDir;
      // Travel
      driveAngle(angle, serialDataSpeed);
      driveDistance(serialDataDist, serialDataSpeed);
      busy = 0;
    } else if (serialDataAction == ACTION_LANE_SWITCH) {
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "\n");
      busy = 1;
      bool dir = false;
      if (receivedChars[1] == 1) {
        dir = true;
      }
      laneswitch(dir);
      busy = 0;
    } else if (serialDataAction == ACTION_INTERSECTION) {
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "\n");
      bool dir = false;
      if (receivedChars[1] == 1) {
        dir = true;
      }
      newData = false;
      busy = 1;
      intersection(dir);
    } else if (serialDataAction == ACTION_SET_MODE) {
      serialDataMode = receivedChars[1];
      mode = (int)serialDataMode;
      //Serial.print((String)mode + "," + (String)receivedChars[0] + "," + (String)receivedChars[1] + "\n");
    }
    newData = false;
  }
  if (mode == LINE_FOLLOWING) {
    linefollowing();
  }
}


//functions for movement
//line following outer sensors (inbetween two lines)
void linefollowing() {
  //custom controls
  // if outside sensors are equal (both white or both black) and not all middle sensors are black -> move straight
  if ((((left_outer.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD)) || ((left_outer.read() < LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD)))
      && !((left.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD)))
  {
    leftSpeed = -SPEED;
    rightSpeed = SPEED;
  }
  // if only left is black -> move to right
  else if ((left_outer.read() > LINETHRESHOLD) && (right_outer.read() < LINETHRESHOLD))
  {
    leftSpeed = -(SPEED);
    rightSpeed = 0;
  }
  // if only right is black -> move to left
  else if ((right_outer.read() > LINETHRESHOLD) && (left_outer.read() < LINETHRESHOLD))
  {
    leftSpeed = 0;
    rightSpeed = SPEED;
  }
  else {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  // if all sensors are on black or up in the air, stop the motors.
  // otherwise, run motors given the control speeds above.
  if ((left.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD) && (right_outer.read() > LINETHRESHOLD) && (left_outer.read() > LINETHRESHOLD) )
  {
    motors.stop();
  }
  else
  {
    motors.leftMotor(leftSpeed);
    motors.rightMotor(rightSpeed);

  }
  delay(0);  // add a delay to decrease sensitivity.
}

//laneswitching
void laneswitch(bool gotoleft) {
  bool lanechanged = false;
  bool angled = false;
  int enccount = 0;
  //Target count is how far the robot will go into the lane?? was 192/4
  int targetCount;
  if (gotoleft) {
    //go left
    unsigned long starttime;
    unsigned long endtime; 
    unsigned long diff;
    starttime = millis();
    while (!lanechanged) {
      motors.rightMotor(SPEED);
      motors.leftMotor(-(SPEED - 30));
      if (right_outer.read() > LINETHRESHOLD) {
        lanechanged = true;
      }
    }
    endtime = millis();
    diff = endtime - starttime;
    while (right_outer.read() > LINETHRESHOLD) {
      motors.drive(SPEED);
    }
    motors.stop();
    targetCount = 192 / 4;
    enccount = encoder.getTicks(LEFT);
    motors.leftMotor(-SPEED);
    targetCount += enccount;
    starttime = millis();
    endtime = millis();
    while (endtime-starttime < diff/2) {
      enccount = encoder.getTicks(LEFT);
      endtime = millis();
    }
  }else{
    //go right
    unsigned long starttime;
    unsigned long endtime; 
    unsigned long diff;
    starttime = millis();
    while (!lanechanged) {
      motors.rightMotor(SPEED-30);
      motors.leftMotor(-(SPEED));
      if (left_outer.read() > LINETHRESHOLD) {
        lanechanged = true;
      }
    }
    endtime = millis();
    diff = endtime - starttime;
    while (left_outer.read() > LINETHRESHOLD) {
      motors.drive(SPEED);
    }
    motors.stop();
    targetCount = 192 / 4;
    enccount = encoder.getTicks(RIGHT);
    motors.rightMotor(SPEED);
    targetCount += enccount;
    starttime = millis();
    endtime = millis();
    while (endtime-starttime < diff/2) {
      enccount = encoder.getTicks(RIGHT);
      endtime = millis();
    }
  }
  //finalize lanechange
  motors.stop();
}


//move into intersection
void intersection(bool gotoleft) {
  int enccount = 0;
  float numRev;
  int targetCount;

  motors.stop();

  if (gotoleft) {
    //turn 90° left
    numRev = 0;
    targetCount = 0;
    enccount = encoder.getTicks(RIGHT);;
    numRev = rotdistintersec / wheelCirc;
    targetCount = numRev * countsPerRev;
    targetCount += enccount;
    while (enccount < targetCount) {
      enccount = encoder.getTicks(RIGHT);
      Serial.println(enccount);
      motors.rightMotor(SPEED);
    }
    motors.rightMotor(0);
  } else {
    //turn 90° right
    numRev = 0;
    targetCount = 0;
    enccount = encoder.getTicks(LEFT);;
    numRev = rotdistintersec / wheelCirc;
    targetCount = numRev * countsPerRev;
    targetCount += enccount;
    while (enccount < targetCount) {
      enccount = encoder.getTicks(LEFT);
      Serial.println(enccount);
      motors.leftMotor(-SPEED);
    }
    motors.leftMotor(0);
  }

  //move until end of intersection
  while (!((left.read() > LINETHRESHOLD) && (right.read() > LINETHRESHOLD))) {
    linefollowing();
  }
  motors.stop();
  getoutofintersection();
}

//wait to get out of intersection (either right or left)
void getoutofintersection() {
  int enccount = 0;
  float numRev;
  int targetCount;

  busy = 0;

  if (waitforinput()) { //true -> move to right, false -> move to left
    motors.leftMotor(0);
    //turn 90° left
    numRev = rotdistintersec / wheelCirc;
    targetCount = numRev * countsPerRev;
    enccount = encoder.getTicks(RIGHT);
    targetCount += enccount;
    while (enccount < targetCount) {
      enccount = encoder.getTicks(RIGHT);
      Serial.println(enccount);
      motors.rightMotor(SPEED);
    }
    motors.rightMotor(0);
  } else {
    //turn 90° right
    numRev = rotdistintersec / wheelCirc;
    targetCount = numRev * countsPerRev;
    enccount = encoder.getTicks(LEFT);
    targetCount += enccount;
    while (enccount < targetCount) {
      enccount = encoder.getTicks(LEFT);
      Serial.println(enccount);
      motors.leftMotor(-SPEED);
    }
  }
}

bool waitforinput() {
  while (!newData) {
    recString();
  }
  newData = false;
  if (receivedChars[1] == 1) {
    return false;
  } else {
    return true;
  }
}

void readdist() {
  // trigger sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //read sensor and calculate distance
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration / 2) * 0.03432;
}

void sendvals() {
  //lCount = encoder.getTicks(LEFT);
  //rCount = encoder.getTicks(RIGHT);
  volatile float lWheelDist = ((lCount / countsPerRev) * wheelCirc);
  volatile float rWheelDist = ((rCount / countsPerRev) * wheelCirc);

  // Write something to serial
  Serial.print((String)busy+","+(String)lCount+","+(String)rCount+","+(String)distance+"\n");
}

void driveDistance(float distance, int motorPower)
{
  long lCount = 0;
  long rCount = 0;
  float numRev;
  int targetCount = 0;
  int enccount = 0;

  numRev = (float) distance / wheelCirc;
  motors.drive(motorPower);

  targetCount = numRev * countsPerRev;
  enccount = encoder.getTicks(RIGHT);
  targetCount += enccount;
  while (enccount < targetCount) {
    enccount = encoder.getTicks(RIGHT);
  }
  motors.stop();
}

void driveAngle(float angle, int motorPower) {
  long lCount = 0;
  long rCount = 0;
  float numRev;
  int targetCount = 0;
  int enccount = 0;

  float rotationDist = (abs(angle) / 360) * PI * 2 * 10;
  numRev = (float) rotationDist / wheelCirc;

  if (angle > 0) {
    motors.leftMotor(motorPower);
    motors.rightMotor(motorPower);
  } else {
    motors.leftMotor(-motorPower);
    motors.rightMotor(-motorPower);
  }

  targetCount = numRev * countsPerRev;
  enccount = encoder.getTicks(RIGHT);
  targetCount += enccount;
  while (enccount < targetCount) {
    enccount = encoder.getTicks(RIGHT);
  }
  motors.stop();
}
