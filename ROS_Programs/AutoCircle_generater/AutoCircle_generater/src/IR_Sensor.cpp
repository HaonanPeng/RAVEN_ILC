
#include "IR_Sensor.h"
#include <iostream>
#include <math.h>
#include <fstream>

// Void constructor
IR_Sensor::IR_Sensor()
{

}

int main(int argc, char **argv)
{
    IR_Sensor Sensor;
    Sensor.init_calib(argc,argv);

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
        distance = sensorVal_to_distance(sensorVal);  //In the initial part, we use the origin sensor value instead of the average
        increaseRate = INC_DECAY*increaseRate+(1-INC_DECAY)*abs(distance-distance_old);
        initial_place = initial_place + distance/INIT_NUM;
        init_counter++;
        std::cout << (init_counter*100/INIT_NUM) <<"%, "<<"initial_place=" << initial_place << std::endl;

        if(init_counter==INIT_NUM)
        {
            initial_place = 10000*calibrator( initial_place/10000 );
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
        t_new_sensor_value = sensorVal;  //[test]
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

        showDistance(); //This should be removed when using, because it is for testing.
        recorder_IR(distance);
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

    if (init_counter> INIT_NUM) y = calibrator(y); // Calibration correction, not activated during initialization
    long double dis = y * 10000; //Transfer to micro meter;

    

    return dis;
}


void IR_Sensor::showDistance()
{
    std::cout<<"----------------"<<std::endl;
    std::cout<<"Distance(cm): "<< distance/10000 <<std::endl;
    std::cout<<"Initial Pleace(cm): "<< initial_place/10000 <<std::endl;
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
    /*
    if(abs(sensorval_origin-sen_val_avr)>2*sen_val_stdev)
    {
        long double dev_para = abs(sensorval_origin-sen_val_avr)/50*sen_val_stdev;  //
        sensorval_filtered = (dev_para*sen_val_avr+sensorval_origin)/(1+dev_para);
    }
    else
    */
    sensorval_filtered = sensorval_origin;
   

    return sensorval_filtered;
}


// This is the function of calibration, which will be the start of the whole program
void IR_Sensor::init_calib(int argc, char **argv)
{
    show_calib_menu();
    int userInput;
    std::cin>> userInput;
    std::cin.get();
    cali_place=0;

    cali_para10cm = 1;  //Set defult values
    cali_para15cm = 1;
    cali_para20cm = 1;

    double cali_para10cm_temp; //Temporary calibration parameters
    double cali_para15cm_temp;
    double cali_para20cm_temp;

    ros::init(argc, argv, "IR_Sensor");

	static ros::NodeHandle cali_n;

   
	IR_sub = cali_n.subscribe("IR_range_data",1,&IR_Sensor::callback_IR_Sensor_Cali,this);


    if (userInput==1) // Quick start, with no calibration
    {
        cali_para10cm = 1;
        cali_para15cm = 1;
        cali_para20cm = 1;
    }
    else if(userInput==2) //Start with calibration
    {
        cali_counter=0;

        // Step 1: 10cm
        int cali_step=10; // The first step is 10 cm
        std::cout<<"Now the calibration begins. Please follow the instruction." << std::endl;
        std::cout<<"Make sure the distance is 10cm, and then press enter." << std::endl;
        std::cin.get();

        while (cali_counter <= CALI_NUM)
        {
            ros::Duration(0.004).sleep();
            ros::spinOnce();

            cali_counter++;
        }
        cali_para10cm_temp = 100000/cali_place;
        std::cout <<"Calibration at:"<< cali_step <<" cm is finished, "<< std::endl;
        std::cout <<"Calibration place(cm):"<< cali_place/10000 << std::endl;
        std::cout <<"Calibration parameter:"<< cali_para10cm_temp << std::endl;
        // Reset ralative variables to prepare for a new step
        cali_counter = 0;
        cali_place = 0;

        // Step 2: 15cm
        cali_step=15; // The second step is 15 cm
        std::cout<<"Make sure the distance is 15cm, and then press enter." << std::endl;
        std::cin.get();

        while (cali_counter <= CALI_NUM)
        {
            ros::Duration(0.004).sleep();
            ros::spinOnce();

            cali_counter++;
        }
        cali_para15cm_temp = 150000/cali_place;
        std::cout <<"Calibration at:"<< cali_step <<" cm is finished, "<< std::endl;
        std::cout <<"Calibration place(cm):"<< cali_place/10000 << std::endl;
        std::cout <<"Calibration parameter:"<< cali_para15cm_temp << std::endl;
        // Reset ralative variables to prepare for a new step
        cali_counter = 0;
        cali_place = 0;

        // Step 3: 20cm
        cali_step=20; // The first step is 10 cm
        std::cout<<"Make sure the distance is 20cm, and then press enter." << std::endl;
        std::cin.get();

        while (cali_counter <= CALI_NUM)
        {
            ros::Duration(0.004).sleep();
            ros::spinOnce();

            cali_counter++;
        }
        cali_para20cm_temp = 200000/cali_place;
        std::cout <<"Calibration at:"<< cali_step <<" cm is finished, "<< std::endl;
        std::cout <<"Calibration place(cm):"<< cali_place/10000 << std::endl;
        std::cout <<"Calibration parameter:"<< cali_para20cm_temp << std::endl;
        // Reset ralative variables to prepare for a new step
        cali_counter = 0;
        cali_place = 0;

        cali_para10cm = cali_para10cm_temp;
        cali_para15cm = cali_para15cm_temp;
        cali_para20cm = cali_para20cm_temp;

        //record the calibration result into a file so that next time if the environment is not changed, there will be no need to calibrate again
        ofstream cali_writer;
        cali_writer.open("cali_para_recorder.txt");
        cali_writer<<cali_para10cm<<' '<<cali_para15cm<<' '<<cali_para20cm<<' '<<std::endl;
        cali_writer.close();

        std::cout <<"Calibration is done, and the result is saved."<< std::endl;
        std::cout <<"Calibration Parameter 10 cm: "<< cali_para10cm <<std::endl;
        std::cout <<"Calibration Parameter 15 cm: "<< cali_para15cm <<std::endl;
        std::cout <<"Calibration Parameter 20 cm: "<< cali_para20cm <<std::endl;

    }
    else if(userInput==3) //Start with last calibration
    {
        ifstream cali_reader;
        cali_reader.open("cali_para_recorder.txt");
        cali_reader>>cali_para10cm;
        cali_reader.get();
        cali_reader>>cali_para15cm;
        cali_reader.get();
        cali_reader>>cali_para20cm;
        cali_reader.get();
        cali_reader.close();

        std::cout <<"Last calibration result loaded:"<< std::endl;
        std::cout <<"Calibration Parameter 10 cm: "<< cali_para10cm <<std::endl;
        std::cout <<"Calibration Parameter 15 cm: "<< cali_para15cm <<std::endl;
        std::cout <<"Calibration Parameter 20 cm: "<< cali_para20cm <<std::endl;

    }
    else
    {
        std::cout<<"[ERROR!] Invalid input, please restart the program."<<std::endl;
        exit(1);
    };

    std::cout<<"Press enter to initialize the sensor." << std::endl;

    
    std::cin.get();

}



void IR_Sensor::show_calib_menu()
{
    using namespace std;
    cout<<"This is IR Sensor main program, if you are not sure about how to use, please check the documentation."<< endl;
    cout<<"There are 3 running option: "<<endl;
    cout<<"-----------------------------------------------------------------------------------"<<endl;
    cout<<"1.[        Quick Start        ]: Start without calibration, max error can be 0.5 cm."<<endl;
    cout<<"2.[   Calibration and Start   ]: Recommended when the environment changes."<<endl;
    cout<<"3.[Start with Last Calibration]: Recommended when the environment dost not change (Not valid for the first time run on the computer)."<<endl;
    cout<<"-----------------------------------------------------------------------------------"<<endl;
    cout<<"Please enter 1, 2 or 3 to choose initialization mode:"<<endl;
}



void IR_Sensor::callback_IR_Sensor_Cali(std_msgs::Int16 range_msg)
{
        int sensorVal = range_msg.data;
        distance = sensorVal_to_distance(sensorVal);  //In the initial part, we use the origin sensor value instead of the average
        cali_place = cali_place + distance/CALI_NUM;
        std::cout <<(cali_counter*100/CALI_NUM) <<"%, "<<"cali_place=" << cali_place <<std::endl;
}

//This is the function which use the calibration parameters to correct the distance
long double IR_Sensor::calibrator(long double origin_distance)
{
    double cali_para;

    if ((origin_distance > 15) && (origin_distance < 20))
    {
        cali_para = cali_para15cm + ((origin_distance-15)/5)*(cali_para20cm - cali_para15cm);
    }
    else if ((origin_distance > 10) && (origin_distance <= 15))
    {
        cali_para = cali_para15cm + ((15-origin_distance)/5)*(cali_para10cm - cali_para15cm);
    }
    else if (origin_distance >= 20)
    {
        cali_para = cali_para20cm;
    }
    else if (origin_distance <= 10)
    {
        cali_para = cali_para10cm;
    };

    std::cout << "cali_para= " <<cali_para << std::endl;  //[test]

    long double calied_distance = cali_para * origin_distance;

    return calied_distance;
}
