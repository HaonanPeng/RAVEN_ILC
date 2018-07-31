#ifndef _TIC_TOC_H
#define _TIC_TOC_H

#include <vector>
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>

class tictoc
{
    private:
    double start_time;
    std::vector<double> toc_time;
    int toc_counter;

    public:
    void tic();
    double toc();

};

void tictoc::tic()
{
    toc_time.resize(1,0);
    start_time= ros::Time::now().toSec();
    std::cout<<"Start time: "<< start_time <<std::endl;
    toc_time[0]=start_time;
    toc_counter=1;
}

double tictoc::toc()
{
    toc_time[toc_counter]=ros::Time::now().toSec()-start_time;
    std::cout<<"Toc time: "<< toc_time[toc_counter] <<std::endl;
    toc_counter++;
    return toc_time[toc_counter-1];
}


#endif