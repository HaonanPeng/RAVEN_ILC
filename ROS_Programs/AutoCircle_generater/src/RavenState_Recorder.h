//
// Created by haonan on 6/19/18.
// In order to use this, include this header. Also goto CMakeLists and add
// add_library(recorder_library RavenState_Recorder.cpp)
// also add this target link to the subscriber executable like
// target_link_libraries(subpose recorder_library ${catkin_LIBRARIES})
// Finally, just add function recorder into the ROS subscriber callback function,
// but notice that the datatype of the input may need to be changed.

#ifndef _RAVENSTATE_RECORDER_H
#define _RAVENSTATE_RECORDER_H

#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include "raven_2/raven_automove.h" // The following 2 hearders can be removed when combining with raven code. They are here to pass catkin_make when testing with turtlesim
#include "raven_2/raven_state.h"

// This is the signal which controls the recorder on(1) or off(0)
#define recorderOn 1

//This path should be changed when running on different computers.
#define turtlePose_FilePath "/home/haonan/catkin_ws/src/antoCircleKinetic/recorderFiles/turtle_pose_recorder.txt"
#define RavenJointPose_FilePath "/home/supernova/test_autocircle/src/AutoCircle_generater/recorderFiles/ravenJoint_pose_recorder.txt"

// This is to define the content of what the recorder will record.
// This will make it easy to change the recorder content.
// Notice that the first column of the text file generated by this recorder is not included. It will always be the time.
#define recorderContent_raven ' '<<msg.jpos[0]<<' '<<msg.jpos[1]<<' '<<msg.jpos[2]<<' '<<msg.jpos[3]<<' '<<msg.jpos[4]<<' '<<msg.jpos[5]<<' '<<msg.jpos[6]<<' '<<msg.jpos[7]<<' '<<msg.jpos_d[0]<<' '<<msg.jpos_d[1]<<' '<<msg.jpos_d[2]<<' '<<msg.jpos_d[3]<<' '<<msg.jpos_d[4]<<' '<<msg.jpos_d[5]<<' '<<msg.jpos_d[6]<<' '<<msg.jpos_d[7]<<' '<<msg.pos[0]<<' '<<msg.pos[1]<<' '<<msg.pos[2]<<' '<<msg.pos_d[0]<<' '<<msg.pos_d[1]<<' '<<msg.pos_d[2]
#define recorderContent_turtle ' ' << msg.x << ' ' << msg.y


using namespace std;

static int recorder_firstCall=0;

static ofstream turtlePose_writer;
static ofstream ravenState_writer;


static double startTime;

void recorder(turtlesim::Pose msg);
void recorder(raven_2::raven_state msg);


#endif //_RAVENSTATE_RECORDER_H