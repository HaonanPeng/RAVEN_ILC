//
// Created by haonan on 6/19/18.
//
#include "RavenState_Recorder.h"



//This function is for turtle_sim in order to test features.
// It will generate a text file which can be loaded into MATLAB,
// where the first column is the time, and the second is x position,
// the third is y position
void recorder(turtlesim::Pose msg)
{
if(recorderOn){
    if(!recorder_firstCall)
    {
        startTime=ros::Time::now().toSec();

        turtlePose_writer.open(turtlePose_FilePath);
        turtlePose_writer<< 0 << recorderContent_turtle <<endl;
        turtlePose_writer.close();

        recorder_firstCall=1;
        turtlePose_writer.open(turtlePose_FilePath, ios::app);
    }
    if(recorder_firstCall) {
        double currTime = ros::Time::now().toSec();
        double relateTime = currTime - startTime;

        turtlePose_writer << relateTime << recorderContent_turtle << endl;
        turtlePose_writer.flush();
        //outf.close();
        if (!ros::ok()) turtlePose_writer.close();
    };
};
}

// This function is to record the raven state.
void recorder(raven_2::raven_state msg)
{
if(recorderOn){
    if(!recorder_firstCall)
    {
        startTime=ros::Time::now().toSec();

        ravenState_writer.open(RavenJointPose_FilePath);
        ravenState_writer<< 0 << recorderContent_raven <<endl;
        ravenState_writer.close();

        recorder_firstCall=1;
        ravenState_writer.open(RavenJointPose_FilePath, ios::app);
    }
    if(recorder_firstCall) {
        double currTime = ros::Time::now().toSec();
        double relateTime = currTime - startTime;

        ravenState_writer << relateTime << recorderContent_raven << endl;
        ravenState_writer.flush();

        if (!ros::ok()) ravenState_writer.close();
    };
};
}


// This is to record the IR sensor
void recorder_IR(long double sensorValue)
{
if(recorderOn){
    if(!recorder_firstCall)
    {
        startTime=ros::Time::now().toSec();

        ravenState_writer.open(IRSensor_FilePath);
        ravenState_writer<< 0 << ' ' <<sensorValue <<endl;
        ravenState_writer.close();

        recorder_firstCall=1;
        ravenState_writer.open(IRSensor_FilePath, ios::app);
    }
    if(recorder_firstCall) {
        double currTime = ros::Time::now().toSec();
        double relateTime = currTime - startTime;

        ravenState_writer << relateTime << ' ' <<sensorValue << endl;
        ravenState_writer.flush();

        if (!ros::ok()) ravenState_writer.close();
    };
};

}