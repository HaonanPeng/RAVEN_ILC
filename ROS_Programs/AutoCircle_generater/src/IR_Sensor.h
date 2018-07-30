/*
This is a program work with IR sensor through Arduino( howerver this program is also independent as it just subscribe to a ros node),
and it will calculate the distance accroding to the sensor signal. It will generate a node with this node one can call a ros service
such as 

ros::ServiceClient client = n.serviceClient<AutoCircle_generater::Sensor_Distance>("get_IR_distance");
AutoCircle_generater::Sensor_Distance sen_dis;
client.call(sen_dis);

and it will return the distance in 'sen_dis.response.distance'
*/
#ifndef IR_Sensor_
#define IR_Sensor_

#include <vector>
#include <numeric>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16.h>
#include <ros/ros.h>
#include "raven_2/Sensor_Distance.h"

// Because the sensor is not very accurate and the value is not very stable, these 2 decay parameter is to deal with inaccuracy.
#define DIS_DECAY 0.1
#define INC_DECAY 0.8

#define SEN_VEC_LEN 10 //This is the lenth of the sensor value vector.
#define INIT_NUM 2000 // This is the number of values used to initiallize the system, should be large in order to get better performance

#define INC_THREASH_HOLD 2

class IR_Sensor
{
    private:
    long double initial_place; // This is the initial place when the program start working. After initiallization, the result of this program will be the relative distance about this initial place.
    long double distance; // This is the current distance
    long double distance_old;  // This is the distance of the one-loop previous distance;
    long double increaseRate;  // Not currently used
    long long int init_counter;  // This is the counter work with initiallzation

    long double t_new_sensor_value; //This is for test

    std::vector<int> sen_val_vec;  // This is the sensor value vector, it stores 10 newest sensor value, in order to deal with the unstability of the sensor
    long double sen_val_avr;  // This is the avrage of the sensor vector
    long double sen_val_stdev; // This is the standard devition of the vector


    ros::Subscriber IR_sub;  // Ros subscriber which subscribe the node generated by the Arduino
    ros::ServiceServer IR_srv;  // Ros service server which provide the service to give the distance required by the Raven side program.


    public:
    IR_Sensor(); // Void Constructor
    void init_ros(int , char**);  // ROS initiallization
    void callback_IR_Sensor(sensor_msgs::Range); // The callback function, main part of this program, it will update the distance and relative values every time the program receives from the Arduino ROS publisher.
    void callback_IR_Sensor_HF(std_msgs::Int16); //Another version of callback function, for high frequency
    long double sensorVal_to_distance(double);  // Transfer sensor value into distance
    void showDistance();  
    bool srvServerFunction(AutoCircle_generater::Sensor_Distance::Request& , AutoCircle_generater::Sensor_Distance::Response& );  // Service server function, return distance to the client
    
    // Following are the functions about sensor value vector, they actially built up a moving average filter
    void vec_addNew(int);
    long double vec_average();
    long double vec_stdev();
    long double sensorVauleStablilizer(int);


    //void setDistance(long double);



};

#endif

/* How to test: 
first, upload program to arduino
$ rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0
$ rosrun AutoCircle_generater IR_Sensor
$ rosrun AutoCircle_generater test_IR_client

*/