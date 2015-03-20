/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

ros::NodeHandle  nh;

//void messageCb( const std_msgs::UInt8& toggle_msg){
//  analogWrite(11, int(toggle_msg.data));   // blink the led
//}

void messageCb( const std_msgs::UInt8MultiArray& toggle_msg){
  analogWrite(11, int(toggle_msg.data[0]));
  analogWrite(10, int(toggle_msg.data[1]));
  analogWrite(9, int(toggle_msg.data[2]));
  analogWrite(6, int(toggle_msg.data[3]));
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("pwm_control", &messageCb );

void setup()
{ 
  pinMode(11, OUTPUT);  //thrust
  pinMode(10, OUTPUT);  //roll
  pinMode(9, OUTPUT);   //pitch
  pinMode(6, OUTPUT);   //yaw
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

