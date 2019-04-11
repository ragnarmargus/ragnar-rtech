/*
   rosserial Publisher Example
   Prints "hello world!"
*/

int echoPin = A4;
int trigPin = A5;


#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle  nh;
sensor_msgs::Range range_msg;
ros::Publisher chatter("ultrasound/raw", &range_msg);

float get_sonar_distance_meters()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration_us = pulseIn(echoPin, HIGH);
  long distance_mm = (duration_us / 58.0) * 10;
  return (float) distance_mm / 1000.0;
}

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  Serial.begin(57600);
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  range_msg.min_range       = 0.02;
  range_msg.max_range       = 4.0;
  range_msg.field_of_view   = 3.14 / 12;
  range_msg.header.frame_id = "range";
}

void loop()
{

  range_msg.range = get_sonar_distance_meters();
  chatter.publish( &range_msg );
  nh.spinOnce();
  delay(100);
}
