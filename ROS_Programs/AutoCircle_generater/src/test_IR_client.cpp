#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <ros/ros.h>
#include "raven_2/Sensor_Distance.h"
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IR_distance_Receiver");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<AutoCircle_generater::Sensor_Distance>("get_IR_distance");

    int i=0;
    while(i<100)
    {
        AutoCircle_generater::Sensor_Distance sen_dis;
        client.call(sen_dis);
        std::cout<<"Received distance(um): "<< sen_dis.response.distance <<std::endl;
        i++;
    };
    
    return 0;
}