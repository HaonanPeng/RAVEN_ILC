/* 
 * rosserial IR Ranger Example  
 * 
 * This example is calibrated for the Sharp GP2D120XJ00F.
 * 
 * [IMPORTANT!!!] If the upload shows 'permision denied', open a terminal and run 'sudo chmod 777 /dev/ttyACM0' 
 */

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>

#define analog_pin A0

ros::NodeHandle  nh;

std_msgs::Int16 Sensorvalue;
//ros::Publisher pub_range( "IR_range_data", &range_msg);
ros::Publisher pub_range( "IR_range_data", &Sensorvalue);


/*
 * getRange() - samples the analog input from the ranger
 * and converts it into meters.  
 */
 
int getRange(int pin_num){
    int sensorValue = analogRead(A0);
    return sensorValue; 
}

char frameid[] = "/ir_ranger";

void setup()
{
  nh.initNode();
  nh.advertise(pub_range);
}

void loop()
{
    //range_msg.range = getRange(analog_pin);
    //range_msg.header.stamp = nh.now();
    //pub_range.publish(&range_msg);
    Sensorvalue.data = getRange(analog_pin);
    pub_range.publish(&Sensorvalue);
    nh.spinOnce();
}

