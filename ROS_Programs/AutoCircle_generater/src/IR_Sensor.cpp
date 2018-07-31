
#include "IR_Sensor.h"
#include <iostream>
#include <math.h>

// Void constructor
IR_Sensor::IR_Sensor()
{

}

int main(int argc, char **argv)
{
    IR_Sensor Sensor;

    Sensor.init_ros(argc,argv);
    
    ros::spin();

    return 0;
}



// ROS initiallization
void IR_Sensor::init_ros(int argc, char **argv)
{
    initial_place=0;
    distance=0;
    distance_old=0;
    increaseRate=0;
    init_counter=0;

    sen_val_vec.resize(SEN_VEC_LEN,0);

    ros::init(argc, argv, "IR_Sensor");

	static ros::NodeHandle n;

    // There are two kinds of callback function, 'IR_Sensor::callback_IR_Sensor' is for low frequency, and 'IR_Sensor::callback_IR_Sensor_HF' is for high frequency
	IR_sub = n.subscribe("IR_range_data",10,&IR_Sensor::callback_IR_Sensor_HF,this); 
    IR_srv = n.advertiseService("get_IR_distance", &IR_Sensor::srvServerFunction,this);

}

// Callback function
void IR_Sensor::callback_IR_Sensor(sensor_msgs::Range range_msg)
{
    
    // Initializing, take the average distance of the first several values and set it as the initial place
    if(init_counter <= 200)
    {
        distance_old = distance;
        double sensorVal = range_msg.range;
        long double tem_distance = sensorVal_to_distance(sensorVal);
        distance = DIS_DECAY * distance + (1 - DIS_DECAY)*tem_distance;
        increaseRate = INC_DECAY*increaseRate+(1-INC_DECAY)*abs(distance-distance_old);
        initial_place = initial_place + distance/200;
        init_counter++;
        std::cout <<"initial_place=" << initial_place << std::endl;

        if(init_counter==200)
        {
            distance=0;
            distance_old=0;
            //increaseRate=0;
            std::cout <<"IR sensor initialized, initial place(um):" << initial_place << std::endl;
            init_counter++;
        };
    }

    // Initialized, normal working
    else
    {
        distance_old = distance;
        double sensorVal = range_msg.range;
        long double tem_distance = sensorVal_to_distance(sensorVal)-initial_place;
        distance = DIS_DECAY * distance + (1 - DIS_DECAY)*tem_distance;

       

        increaseRate = INC_DECAY*increaseRate+(1-INC_DECAY)*abs(distance-distance_old);
         //showDistance(); //[test] This should be removed when using, because it is for testing.
    };  
}


// Another Callback function for high frequency(msg type has changed)
void IR_Sensor::callback_IR_Sensor_HF(std_msgs::Int16 range_msg)
{
    
    // Initializing, take the average distance of the first several values and set it as the initial place
    if(init_counter <= INIT_NUM)
    {
        distance_old = distance;
        int sensorVal = range_msg.data;
        vec_addNew(sensorVal);
        distance = sensorVal_to_distance(sen_val_avr);
        increaseRate = INC_DECAY*increaseRate+(1-INC_DECAY)*abs(distance-distance_old);
        initial_place = initial_place + distance/INIT_NUM;
        init_counter++;
        std::cout <<"initial_place=" << initial_place << std::endl;

        if(init_counter==INIT_NUM)
        {
            distance=0;
            distance_old=0;
            //increaseRate=0;
            std::cout <<"IR sensor initialized, initial place(cm):" << initial_place/10000 << std::endl;
            init_counter++;
        };
    }

    // Initialized, normal working
    else
    {
        distance_old = distance;
        int sensorVal = range_msg.data;
        t_new_sensor_value = sensorVal;
        sensorVal = sensorVauleStablilizer(sensorVal); // If the sensor value is far away from current value, compensate it
        vec_addNew(sensorVal);
        distance = sensorVal_to_distance(sen_val_avr)-initial_place;
        increaseRate = INC_DECAY*increaseRate+(1-INC_DECAY)*abs(distance-distance_old);

         /*/[test] to test the delay caused by the filter
        if ((distance/10000) >0.5 && ticFirstcall==0) 
        {
            tictoc1.tic();
            ticFirstcall=1;
            std::cout <<"--------" << std::endl; //[test]
        };
        if ((distance/10000) >1.5 && ticFirstcall==1)
        {
            tictoc1.toc();
        };
        /[test]end*/

        //showDistance(); //This should be removed when using, because it is for testing.
    };
}

// Service server function, return the distance.
bool IR_Sensor::srvServerFunction(AutoCircle_generater::Sensor_Distance::Request& req, AutoCircle_generater::Sensor_Distance::Response& res)
{
    res.distance=distance;
    return true;
}

// Take sensor value as input and return distance (in micro meter)
long double IR_Sensor::sensorVal_to_distance(double x)
{
    //Currently not uesd. This is the result of a 15th-order polyfit, which will transfer sensor value (under defult referance voltage) to distance(cm)
    //long double y=-121348.695728197+6395.09589115474*x-150.387079166702*pow(x,2)+2.09410051688887*pow(x,3)-0.0192880929486053*pow(x,4)+0.000124220838323152*pow(x,5)-5.75822589460873*pow(10,-7)*pow(x,6)+1.94376654540714*pow(10,-9)*pow(x,7)-4.75911163496137*pow(10,-12)*pow(x,8)+8.24207632536036*pow(10,-15)*pow(x,9)-9.42021682416840*pow(10,-18)*pow(x,10)+5.60137295554236*pow(10,-21)*pow(x,11)+1.07984452527315*pow(10,-24)*pow(x,12)-4.76403623087764*pow(10,-27)*pow(x,13)+3.44305202883701*pow(10,-30)*pow(x,14)-8.99799019219652*pow(10,-34)*pow(x,15);

    //Currently used. This is a low order result. Only valid in the range from 10 cm to 20 cm. When use this, make sure that
    //the result is between 10 and 20 cm.
    long double y=1.18270050154854*pow(10,-9)*pow(x,4)+(-2.25504856400793*pow(10,-6)*pow(x,3))+0.00165150940050058*pow(x,2)+(-0.574424912519396*x)+92.8958757034745;

    long double dis = y * 10000; //Transfer to micro meter;

    return dis;
}

void IR_Sensor::showDistance()
{
    std::cout<<"----------------"<<std::endl;
    std::cout<<"Distance(cm): "<< distance/10000 <<std::endl;
    std::cout<<"IncreaseRate(cm): "<< increaseRate/10000 <<std::endl;
    std::cout<<"Average sensor value: "<< sen_val_avr <<std::endl;
    std::cout<<"New sensor value: "<< t_new_sensor_value <<std::endl;
    std::cout<<"Stdev of sensor value: "<< sen_val_stdev <<std::endl;
    std::cout<<"----------------"<<std::endl;
}

// This is the function to add new menber to the sensor value vector, it will update the newest value to the last menber of the vector,
// and thus push the vector forward by 1 member, which will delete the oldest one
void IR_Sensor::vec_addNew(int x)
{
    for(int i=0; i<=SEN_VEC_LEN-2; i++ )
    {
        sen_val_vec[i]=sen_val_vec[i+1];
    }
    sen_val_vec[SEN_VEC_LEN-1]=x;

    vec_average(); //update the average and the stdev
    vec_stdev();
}

// This function will update the class member 'sen_val_avr' and it will also return the new average
long double IR_Sensor::vec_average()
{
    long double sum = std::accumulate(sen_val_vec.begin(), sen_val_vec.end(), 0.0);
	long double avr = sum / sen_val_vec.size();
    sen_val_avr = avr;
    return avr;
}

// Similar as average
long double IR_Sensor::vec_stdev()
{
    long double sum = std::accumulate(sen_val_vec.begin(), sen_val_vec.end(), 0.0);
	long double mean = sum / sen_val_vec.size();

	long double accum = 0.0;
	for (int i = 0; i <= SEN_VEC_LEN-1; i++)
	{
		accum += (sen_val_vec[i] - mean)*(sen_val_vec[i] - mean);
	};
	long double stdev = sqrt(accum / (sen_val_vec.size() - 1));
    if (stdev<=0.1) stdev=0.1; // In case that stdev=0
    sen_val_stdev = stdev;
    return stdev;
}

// This function is to deal with the sensor values that are far away from normal value
long double IR_Sensor::sensorVauleStablilizer(int sensorval_origin)
{
    long double sensorval_filtered = sen_val_avr; // initiallize as the average value in order to be safer
    if(abs(sensorval_origin-sen_val_avr)>2*sen_val_stdev)
    {
        long double dev_para = abs(sensorval_origin-sen_val_avr)/2*sen_val_stdev;  //
        sensorval_filtered = (dev_para*sen_val_avr+sensorval_origin)/(1+dev_para);
    }
    else
    sensorval_filtered = sensorval_origin;

    return sensorval_filtered;
}
