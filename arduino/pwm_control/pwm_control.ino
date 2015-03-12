/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/UInt8.h>

ros::NodeHandle  nh;

void messageCb( const std_msgs::UInt8& toggle_msg){
  analogWrite(11, int(toggle_msg.data));   // blink the led
}

ros::Subscriber<std_msgs::UInt8> sub("pwm_control", &messageCb );

void setup()
{ 
  pinMode(11, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}

