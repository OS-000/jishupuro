
/// ultrasonic_sensor_sample.ino ///
  
// HC-SR04 Ultrasonic sensor
// https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor

/*
roscore
source ~/enshu_ws/devel/setup.bash
cd ~/Arduino/libraries/
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
roslaunch mechatrobot mechatrobot_driver.launch
 */

#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

#define TRIG_PIN 9
#define ECHO_PIN 10

std_msgs::Float64 distance_msg;
ros::Publisher distance_publisher("arduino/distance", &distance_msg);

float duration, distance;


void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Serial.begin(9600);

  nh.initNode();
  delay(1000);
  nh.advertise(distance_publisher);
}


void loop() {
  // calculate distance
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  
  if(duration>0) {
    distance = (duration*.0343)/2; // ultrasonic speed is 340m/s = 0.034cm/us
    Serial.print(duration);
    Serial.print(" us ");
    Serial.print(distance);
    Serial.println(" cm");

    distance_msg.data = distance;
    distance_publisher.publish(&distance_msg);
  }

  nh.spinOnce();
  delay(200);

  Serial.println("");
}
