/*
	ELE418 - Robot Intelligence.
	Header file containing definitions of the Microbot class.
	This class contains data members and functions dealing
	with the Microbot, under development for the Simubot Project.

	Last revision date : 2-4-99 by Jamie Stultz
	Last revision date : 10-11-2018 by Scott Harding: Replaced all Pointer function parms with type Reference
		so function args do not need to be passed using address-of "&" operator, e.g., InverseKinematics(t,&j) becomes
		InverseKinematics(t,j).

*/
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "serial.h"
#include <iostream>
#include <sstream>



#define OUT_OF_WORKSPACE -1
#define BASE_ANGLE_EXCEEDED -2
#define SHOULDER_ANGLE_EXCEEDED -3
#define ELBOW_ANGLE_EXCEEDED -4
#define PITCH_ANGLE_EXCEEDED -5
#define ROLL_ANGLE_EXCEEDED -6
#define FINGER_HITS_BODY -7
#define WRIST_HITS_BODY -8
#define FINGER_HITS_TABLE -9
#define WRIST_HITS_TABLE -10
#define FINGER_HITS_BASE -11
#define GRIPPER_WIDTH -12
#define	LL 85.0 //96.5 //3.8 inches
#define L 177.8 //7.0 inches 
#define H 195.0 // 210.0// 7.68 inches
#define R1 1.0
#define HAND_WIDTH 38.1 //1.5 inches
#define WRIST_WIDTH 15.24 //0.6 inches
#define MAX_GRIP_WIDTH 63.5 //2.5 inches
#define BODY_RADIUS 50.8 //2 inches
#define BASE_HEIGHT 38.1 //1.5 inches 
#define BASE_X 50.8 //2 inces
#define BASE_Y 76.2 //3 inches
#define ROBOT_BASE_HEIGHT 254.0 //10 inches
#define PI 3.141592653589793238462643383279502884197169399375105820974944
#define CONVER (PI/180.0)
#define BASE_STEPS 1125.543757545884 //7072
#define SHOULDER_STEPS 1125.543757545884 //7072
#define ELBOW_STEPS 661.7662533761008 //4158
#define RIGHT_STEPS 244.4619925891512 //1536
#define LEFT_STEPS 244.4619925891512 //1536
#define GRIPPER_STEPS 14.763779527559056 //375




// Public Data Structures
struct Taskspace
{
	double x, y, z, p, r, g;
}; typedef struct Taskspace Taskspace;

struct Jointspace
{
	double j[7];
}; typedef struct Jointspace Jointspace;

struct Registerspace
{
	int r[9];
}; typedef struct Registerspace Registerspace;

struct Pose
{
	Taskspace ts;
	Jointspace js;
	Registerspace rs;
}; typedef struct Pose Pose;

struct Cube
{
	int n;
	int size;
	Taskspace ts;
	Jointspace js;
	Registerspace rs;
}; typedef struct Cube Cube;

struct Tower
{
	int n;
	int height;
	Taskspace ts;
	Jointspace js;
	Registerspace rs;
}; typedef struct Tower Tower;


struct IOspace
{
	int lastkey;
	int enc;
	int open;
	int ctrl[4];
	int latch;
	int reg;
}; typedef struct IOspace IOspace;
class Microbot
{
public:

	// Constructors
	Microbot();
	Microbot(Taskspace home);

	// Public Member Functions
	int MoveTo(Taskspace &t);
	int Error(int);
	void SetSpeed(int);
	void CurrentPosition(Taskspace &t);
	void CurrentPosition(Jointspace &j);
	void CurrentPosition(Registerspace &r);
	int SendStep(int speed, Registerspace delta);
	int SendClose(int speed, int force);
	int SendRead(Registerspace &read);
	int SendSet(int speed);
	int SendReset();
	int SetDelta(Registerspace start, Registerspace finish);
	int SetDelta(Jointspace start, Jointspace finish);
	void GoHome();

	int CheckWorkspaceLimits(Jointspace j);
	bool CheckWorkspaceLimits(Taskspace t);

	int PickandPlace(Taskspace start, Taskspace finish);



private:

	// Private Data Members
	CSerial port;
	Pose home;
	Pose currentPose;
	Pose lastPose;
	Jointspace deltaJoints;
	Registerspace delta;
	int microbot_speed;

	// Private Utility Member Functions
	double ABS(double);
	int ROUND(double);

	// Private Kinematic Member Functions
	void SetTaskspace(Taskspace t);
	void SetJointspace(Jointspace j);
	void SetRegisterspace(Registerspace r);
	int InverseKinematics(Taskspace t, Jointspace &j);
	int ForwardKinematics(Jointspace j, Taskspace &t);
	int JointToRegister(Jointspace j, Registerspace &r);
	int RegisterToJoint(Registerspace r, Jointspace &j);

	int SpaceConvertion(Pose &pose, Taskspace t);
	int SpaceConvertion(Pose &pose, Registerspace r);
	int SpaceConvertion(Pose &pose, Jointspace j);
};
