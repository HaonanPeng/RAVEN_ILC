#ifndef RAVEN_PATHPLANNER_H_
#define RAVEN_PATHPLANNER_H_

/*
[IMPORTANT!!!]
Please notice that this version has a line movement feature. Change 'LineMovementOn' into 1 to turn it on, and into 0 to turn it off.
The program share the same radius with circle movement. The tip will move along x, y or z axis, with the mid point to be the center after
ros is initiallized (please do not change the center when running), with the length to be double radius. The only two changes that can be made 
is changing radius and speed. The moving axis can be changed in the constuctor of this class. The initial speed and radius can be changed in 
'Raven_Controller.cpp' in member function init_sys().
*/

#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sstream>
#include <pthread.h>
#include <termios.h>
#include <queue>
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"

#define LEFT_ARM 0
#define RIGHT_ARM 1

#define MAX_RADIUS 6   // 6 different radius levels
#define MAX_SPEED 60   // 60 different speed levels
#define MIN_RADIUS 1
#define MIN_SPEED 1
#define SMALL_RADIUS 2
#define SMALL_RADIUS_MAX_SPEED 50
#define VERTICLE_CIRCLE_MAX_SPEED 40
#define CHANGE_BASEPLANE_MAX_SPEED 10

#define RADIUS_level_TO_microm 3000  //in micro meter (= 3mm = 0.3cm)

#define DEL_POS_THRESHOLD 180	// in micro meter (= 0.18mm = 0.018cm)
#define STATE_THRESHOLD 300
#define DEL_ROT_THRESHOLD 0.25 	// in degrees
#define ROS_PUBLISH_RATE 1000 	// in Hz

#define YZ_PLANE 0    // circluate around X axis
#define XZ_PLANE 1    // circluate around Y axis
#define XY_PLANE 2    // circluate around Z axis

// [DANGER]: for modification parameter tuning only
#define DEFAULT_MODIFICATION_SCALE  	  	0.000001
#define DEFAULT_MODIFICATION_SPEED_POWER  	0.7
#define DEFAULT_MODIFICATION_DISTANCE_POWER  	1

// [Line movement] Line movement start here
#define LineMovementOn 1 
// This is the signal to turn on or off the line movement. 1 = on, 0 = off

#define Li_MaxSp 30 
// This is the max speed, it should not be larger than 180, but here it is set to be 30 in order to be safer.

// Following is the three possible derection of line movement. Need to be changed before running, currently cannot be changed when running
#define Li_X_AXIS 0
#define Li_Y_AXIS 1
#define Li_Z_AXIS 2
// [Line movement] end here



using namespace raven_2;
using namespace std;


enum PATH_STATE{
	AROUND_CIRCLE,	// go along circle trajectory
	MOVETO_CIRCLE	// move to the new circle (when radius or center change)
};


class Raven_PathPlanner
{
	private:
		tfScalar Modi_Scale;
		tfScalar Modi_Speed_Pow;
		tfScalar Modi_Dista_Pow;
		
		tf::Vector3 X_AXIS;
		tf::Vector3 Y_AXIS;
		tf::Vector3 Z_AXIS;

		tf::Vector3 Center;		// the center of the circle
						// [NOTE]: good center (-85126,-22305,43358)
		tf::Vector3 Current_Pos;	// current raven position
		tf::Vector3 Delta_Pos;
		tf::Quaternion Current_Ori;	// current raven rotation

		tfScalar Radius;		// in mm
		tfScalar Speed;
		tfScalar Distance;		// the distance between current pos and center
		tfScalar Error;			// the difference between radius and distance
		PATH_STATE PathState;
	
		pthread_mutexattr_t data1MutexAttr;
		pthread_mutex_t data1Mutex;

		int Direction;
		int Base_Plane;
		int ArmType;
		bool FIRST_SEND;
		tfScalar last_y,last_z;
		tfScalar K;
		tfScalar Kp;
		tfScalar sign(tfScalar);
		void checkPathState();
		void AutoCircleMotion1();		// algorithm 1 : kind of unstable
		void AutoCircleMotion2();		// algorithm 2 : better!
		void AutoCircleMotion3();		// algorithm 3 : even better! 
		tf::Vector3 AutoCircleMotion4();	// algorithm 4 : the best so far! (in use!!)	
		tf::Vector3 TuneRadiusMotion();

		// [Line movement] Line movement start here.
		int Li_Base_Plane;  // This is the line that the tip will follow, should be the x,y,z axis
		tfScalar Li_Direction;  //This can only be 1, 0, -1, which denote for the forward or backward movement along the axis
		// [Line movement] end here
			

	public:
		Raven_PathPlanner();
		bool set_Radius(int);
		bool set_Speed(int);
		bool set_Direction(int);
		bool set_BasePlane(int);
		bool set_ArmType(int);
		bool set_Center(boost::array<int, 6>);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);
		tfScalar get_Radius();
		tfScalar get_Radius_Range();
		tfScalar get_Speed();
		tfScalar get_K();
		void show_PathState();
		void show_Distance();
		void show_Center();
		void show_delPos();
/*
		// [DANGER]: for modification parameter tuning only
		bool set_Modi_Scale(int);
		bool set_Modi_Speed_Pow(int);
		bool set_Modi_Dista_Pow(int);
		tfScalar get_Modi_Scale();
		tfScalar get_Modi_Speed_Pow();
		tfScalar get_Modi_Dista_Pow();
*/

		tfScalar DistanceOf(tf::Vector3,tf::Vector3);
		tf::Transform ComputeCircleTrajectory();
		tf::Transform ComputeNullTrajectory();

// [Line movement] Line movement start here ,
// Defined at 'Raven_PathPlanner.cpp' [line 1076]
		tf::Transform ComputeLineTrajectory(); //This is the line version of 'ComputeCircleTrajectory()', the main part of line movement
// [Line movement] end here
};
#endif