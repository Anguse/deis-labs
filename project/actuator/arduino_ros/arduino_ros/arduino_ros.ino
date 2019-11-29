#include <RedBot.h>
#include <RedBotSoftwareSerial.h>
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Illuminance.h>
#include <TimerOne.h>

// Sonar
#define TRIG_PIN 11
#define ECHO_PIN A5

/*
   rosserial Servo Control Example

   This sketch demonstrates the control of hobby R/C servos
   using ROS and the arduiono

   For the full tutorial write up, visit
   www.ros.org/wiki/rosserial_arduino_demos

   For more information on the Arduino Servo Library
   Checkout :
   http://www.arduino.cc/en/Reference/Servo
*/

// IOs
RedBotMotors motors;
RedBotSensor left_outer = RedBotSensor(A0);
RedBotSensor left = RedBotSensor(A3);
RedBotSensor right = RedBotSensor(A7);
RedBotSensor right_outer = RedBotSensor(A6);
RedBotEncoder encoder = RedBotEncoder(A2, A4);

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

//geometry_msgs::TransformStamped t;
//tf::TransformBroadcaster broadcaster;
ros::NodeHandle  nh;

double x = 1.0;
double y = 0.0;
double theta = 1.57;

char base_link[] = "/base_link";
char odom[] = "/odom";

int LINETHRESHOLD = 700;
bool LINEFOLLOW = false;
int SPEED = 50;

// Callbacks
void linefollow_cb( const std_msgs::Int16& cmd_msg) {
  if (cmd_msg.data > -1) {
    LINEFOLLOW = true;
    SPEED = cmd_msg.data;
  } else {
    LINEFOLLOW = false;
    SPEED = 0;
  }
}
void leftWheel_cb( const std_msgs::Int16& cmd_msg) {
  motors.leftMotor(-cmd_msg.data);
}
void rightWheel_cb( const std_msgs::Int16& cmd_msg) {
  if (cmd_msg.data > 0) {
    motors.rightMotor(cmd_msg.data + 3);
  } else if (cmd_msg.data < 0) {
    motors.rightMotor(cmd_msg.data - 3);
  } else {
    motors.rightMotor(cmd_msg.data);
  }
}

//sensor_msgs::Range range_msg;
sensor_msgs::Illuminance illu_left_msg, illu_right_msg;//, illu_left_inner_msg, illu_right_inner_msg;
std_msgs::Int16 linefollow_msg;
//ros::Publisher pub_range( "bigboy/ultrasound", &range_msg);
ros::Publisher pub_left( "bigboy/left", &illu_left_msg);
ros::Publisher pub_right( "bigboy/right", &illu_right_msg);
ros::Publisher pub_linefollow( "bigboy/linefollow", &linefollow_msg);
//ros::Publisher pub_left_inner( "bigboy/left_inner", &illu_left_inner_msg);
//ros::Publisher pub_right_inner( "bigboy/right_inner", &illu_right_inner_msg);

ros::Subscriber<std_msgs::Int16> lw_sub("bigboy/arduino/leftWheel", leftWheel_cb);
ros::Subscriber<std_msgs::Int16> rw_sub("bigboy/arduino/rightWheel", rightWheel_cb);
ros::Subscriber<std_msgs::Int16> linefollow_sub("bigboy/arduino/linefollow", linefollow_cb);

void setup() {
  encoder.clearEnc(BOTH);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  nh.initNode();
  //broadcaster.init(nh);
  //nh.advertise(pub_range);
  nh.advertise(pub_left);
  nh.advertise(pub_right);
  //nh.advertise(pub_left_inner);
  //nh.advertise(pub_right_inner);
  /*
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.header.frame_id =  "/ultrasound";
    range_msg.field_of_view = 0.1;  // fake
    range_msg.min_range = 0.0;
    range_msg.max_range = 6.47;
  */

  nh.subscribe(lw_sub);
  nh.subscribe(rw_sub);
  nh.subscribe(linefollow_sub);
}

void loop() {
  //updateOdom();
  //updateDist();
  updateIllu();
  if (LINEFOLLOW) {
    linefollowing();
  }
  nh.spinOnce();
  //delay(1);
}
void linefollowing() {
  //custom controls
  // if outside sensors are equal (both white or both black) and not all middle sensors are black -> move straight
  int leftSpeed, rightSpeed;
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
/*
  void updateOdom(){
  // drive in a circle
  double dx = 0.2;
  double dtheta = 0.18;
  x += cos(theta)*dx*0.1;
  y += sin(theta)*dx*0.1;
  theta += dtheta*0.1;
  if(theta > 3.14)
    theta=-3.14;

  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  t.transform.translation.x = x;
  t.transform.translation.y = y;

  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();

  broadcaster.sendTransform(t);
  }
*/
/*
  void updateDist(){
  // trigger sensor
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  //read sensor and calculate distance
  long duration = pulseIn(ECHO_PIN, HIGH);
  long distance = (duration / 2) * 0.03432;

  range_msg.range = distance;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  }
*/
void updateIllu() {
  illu_left_msg.illuminance = left_outer.read();
  //illu_left_inner_msg.illuminance = left.read();
  illu_right_msg.illuminance = right_outer.read();
  //illu_right_inner_msg.illuminance = right.read();
  illu_left_msg.header.stamp = nh.now();
  //illu_left_inner_msg.header.stamp = nh.now();
  illu_right_msg.header.stamp = nh.now();
  //illu_right_inner_msg.header.stamp = nh.now();
  pub_left.publish(&illu_left_msg);
  //pub_left_inner.publish(&illu_left_inner_msg);
  pub_right.publish(&illu_right_msg);
  //pub_right_inner.publish(&illu_right_inner_msg);
}
