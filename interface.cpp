/*
ELE418 - Robotics.
interface.cpp
	Last revision date : 2-4-99 : Jamie Stultz
	Last revision date : 10-10-2018 : Scott Harding
		1. Replaced all _itoa() function calls with ISO C++11 STL String functions
		2. Replaced all parameters in functions of type Pointer to type Reference
			so all code using a struct must use "." dot operator to reference
			struct data instead of "->" arrow.
*/
#include <string>
#include "kinematics.h"
using namespace std;

using String = std::string;
Microbot::Microbot() {

	port.Open(1, 9600);
	microbot_speed = 234;
	SendReset();

	home.ts.x = 125;
	home.ts.y = 0;
	home.ts.z = 0;
	home.ts.p = -90;
	home.ts.r = 0;
	home.ts.g = 0;
	InverseKinematics(home.ts, home.js);
	JointToRegister(home.js, home.rs);

	currentPose = home;
	lastPose = home;

};

int Microbot::SendStep(int speed, Registerspace del)
{
	int i;
	int ret_num = 0;
	double timer = 1000000;
	char ret[2];
	char c[80] = "@STE ";

	String spe{ std::to_string(speed) };
	String m1{ std::to_string(del.r[1]) };
	String m2{ std::to_string(del.r[2]) };
	String m3{ std::to_string(del.r[3]) };
	String m4{ std::to_string(del.r[4]) };
	String m5{ std::to_string(del.r[5]) };
	String m6{ std::to_string(del.r[6]) };
	String m7{ std::to_string(del.r[7]) };


	strcat(c, spe.c_str());
	strcat(c, ",");
	strcat(c, m1.c_str());
	strcat(c, ",");
	strcat(c, m2.c_str());
	strcat(c, ",");
	strcat(c, m3.c_str());
	strcat(c, ",");
	strcat(c, m4.c_str());
	strcat(c, ",");
	strcat(c, m5.c_str());
	strcat(c, ",");
	strcat(c, m6.c_str());
	strcat(c, ",");
	strcat(c, m7.c_str());
	strcat(c, "\r");

	// print statement for debugging; may be commented out afterwards
	printf("String = %s\n", c);

	fflush(stdin);

	i = strlen(c);
	port.SendData(c, i);


	while ((ret_num == 0) && (timer > 0))
	{
		ret_num = port.ReadData(ret, 2);
		timer = (timer - 0.25);
	}

	if (timer <= 0)
		printf("Error\n");

	if (ret_num != 0)
	{
		// print statement for debugging; may be commented out afterwards
		printf("Return from Microbot: %c\n", ret[0]);
		ret[1] = '\0';
		i = atoi(ret);
	}

	return i;

}


int Microbot::SendClose(int speed, int force)
{

	int i;
	int ret_num = 0;
	char ret[2];
	char c[80] = "@CLO ";

	Registerspace r;

	String spe = std::to_string(speed);

	if (speed != -1)
	{
		strcat(c, spe.c_str());

	}

	strcat(c, "\r");

	printf("String = %s\n", c);

	fflush(stdin);

	i = strlen(c);

	port.SendData(c, i);

	i = 0;

	while (i == 0)
		i = port.ReadData(ret, 2);

	i = 1;

	if (force != -1)
	{

		if (speed == -1)
		{
			speed = 221;
		}

		r.r[1] = 0;
		r.r[2] = 0;
		r.r[3] = 0;
		r.r[4] = 0;
		r.r[5] = 0;
		r.r[6] = -10 * force;

		i = SendStep(speed, r);
	}

	return i;

}



int Microbot::SendRead(Registerspace &read)
{
	int	i, count = 1;
	int a = 0;
	int ret_num = 0;


	char c[10] = "@READ\r";
	char d[80];

	i = strlen(c);
	port.SendData(c, i);

	while (ret_num == 0)
	{

		ret_num = port.ReadData(d, 1);

	}


	while (a < 2)
	{
		ret_num = 0;

		while (ret_num == 0)
			ret_num = port.ReadData(d + count, 1);

		if (d[count] == '\r')
			a++;

		count++;

	}

	d[count - 1] = '\0';

	int index = 2;
	int index1 = 1;

	while (d[index] != '\0')
	{

		read.r[index1] = atoi(d + index);

		index1++;

		while (d[index] != ','&&d[index] != '\0')
			index++;

		if (d[index] == ',')
			index++;
	}

	printf("Motor steps in ascii =  %s\n", d + 2);



	return 0;
}


int Microbot::SendSet(int speed)
{
	int	i;
	int ret_num = 0;
	char ret[2];
	char c[15] = "@SET";

	std::string spe = std::to_string(speed);

	strcat(c, spe.c_str());
	strcat(c, ",");
	strcat(c, "\r");

	printf("String = %s\n", c);

	i = strlen(c);
	port.SendData(c, i);

	while (ret_num == 0)
		ret_num = port.ReadData(ret, 2);

	ret[1] = '\0';
	i = atoi(ret);

	return i;
}

int Microbot::SendReset()
{

	int i;
	int ret_num = 0;
	char ret[2];
	char c[80] = "@RESET ";


	strcat(c, "\r");

	printf("String = %s\n", c);

	fflush(stdin);

	i = strlen(c);

	port.SendData(c, i);

	i = 0;

	while (i == 0)
		i = port.ReadData(ret, 2);

	i = 1;
	return i;

}
//############################# Our Stuff #################################
void Microbot::SetSpeed(int newSpeed) {
	microbot_speed = newSpeed;
};


int Microbot::InverseKinematics(Taskspace p, Jointspace &j) {
	double X = p.x;
	double Y = p.y;
	double Z = p.z;
	double P = p.p * PI / 180.0;
	double R = p.r * PI / 180.0;
	double G = p.g;
	double RR, R0, Z0, b, h, beta, alpha;
	Jointspace tmp = j;

	RR = sqrt(X*X + Y * Y);
	R0 = RR - LL * cos(P);
	Z0 = Z - LL * sin(P) - H;
	b = 0.5 * sqrt(R0*R0 + Z0 * Z0);
	h = sqrt(L*L - b * b);
	if (isnan(h)) {
		printf("ERROR: ATTEMTING TO MOVE OUTSIDE WORKSPACE \n");
		return 0;
	};


	alpha = atan2(h, b);
	beta = atan2(Z0, R0);

	//cout << " X = " << X << " Y = " << Y << " Z = " << Z << " RR = " << RR << " R0 = " << R0 << " Z0 = " << Z0 << " b = " << b << " h = " << h << " alpha = " << alpha << " beta = " << beta <<  endl;


	tmp.j[1] = atan2(Y, X);
	tmp.j[2] = alpha + beta;
	tmp.j[3] = beta - alpha;
	tmp.j[4] = P - R - R1 * tmp.j[1];
	tmp.j[5] = P + R + R1 * tmp.j[1];

	tmp.j[6] = G;

	int check = CheckWorkspaceLimits(tmp);
	if (check < 0) {
		return 0;
	};

	j = tmp;


	return 1;
};

int Microbot::ForwardKinematics(Jointspace j, Taskspace &t) {
	double RR;
	t.p = ((j.j[5] + j.j[4]) / 2.0 + R1 * j.j[1]) * (180.0 / PI);
	t.r = ((j.j[5] - j.j[4]) / 2.0 - R1 * j.j[1]) * (180.0 / PI);
	RR = (L * cos(j.j[2]) + L * cos(j.j[3]) + LL * cos(t.p));

	t.x = RR * cos(j.j[1]);
	t.y = RR * sin(j.j[1]);
	t.z = H + L * sin(j.j[2]) + L * sin(j.j[3]) + LL * sin(t.p);

	t.g = j.j[6];

	//R = R * 180 / pi
	//P = P * 180 / pi
	// R1 = R1 * 180 / pi

	return 1;
};

int Microbot::JointToRegister(Jointspace j, Registerspace &r) {

	r.r[1] = int(round(j.j[1] * BASE_STEPS)); //1125.0); //Base
	r.r[2] = int(-round(j.j[2] * SHOULDER_STEPS)); //1125.0); //Shoulder
	r.r[3] = int(-round(j.j[3] * ELBOW_STEPS)); //672.0); //Elbow
	r.r[4] = int(-round(j.j[4] * RIGHT_STEPS)); //244.4); //Wrist 1
	r.r[5] = int(-round(j.j[5] * LEFT_STEPS)); //244.4); //Wrist 2
	r.r[6] = int(-round(j.j[3] * ELBOW_STEPS - j.j[6] * GRIPPER_STEPS)); //14.76); //Gripper

	return 1;
};


int Microbot::RegisterToJoint(Registerspace r, Jointspace &j) {
	j.j[1] = r.r[1] / BASE_STEPS; //Base
	j.j[2] = -r.r[2] / SHOULDER_STEPS; //Shoulder
	j.j[3] = -r.r[3] / ELBOW_STEPS; //
	j.j[4] = -r.r[4] / RIGHT_STEPS;
	j.j[5] = -r.r[5] / LEFT_STEPS;
	j.j[6] = (r.r[6] - r.r[3]) / GRIPPER_STEPS;

	return 1;

}


int Microbot::CheckWorkspaceLimits(Jointspace j) {

	// Base
	if (j.j[1] * 180.0 / PI > 90 || j.j[1] * 180.0 / PI < -90) {
		printf("BASE ANGLE EXCEEDED \n");
		return BASE_ANGLE_EXCEEDED;
	};
	// Shoulder
	if ((j.j[2] * 180.0 / PI > 144) || (j.j[2] * 180.0 / PI < -35)) {
		printf("SHOULDER_ANGLE_EXCEEDED \n");
		return SHOULDER_ANGLE_EXCEEDED;
	};
	// Elbow
	if (j.j[3] * 180.0 / PI > 0 || j.j[3] * 180.0 / PI < -149) {
		printf("ELBOW_ANGLE_EXCEEDED \n");
		return ELBOW_ANGLE_EXCEEDED;
	};
	// Wrist Pitch
	if ((j.j[5] + j.j[4])*0.5 * 180.0 / PI > 90 || (j.j[5] + j.j[4])*0.5 * 180.0 / PI < -90) {
		return PITCH_ANGLE_EXCEEDED;
		printf("PITCH_ANGLE_EXCEEDED \n");
	};

	// Wrist Roll
	if ((j.j[5] - j.j[4])*0.5 * 180.0 / PI > 270 || (j.j[5] - j.j[4])*0.5 * 180.0 / PI < -270) {
		return ROLL_ANGLE_EXCEEDED;
		printf("ROLL_ANGLE_EXCEEDED \n");
	};

	return 1;

}

bool Microbot::CheckWorkspaceLimits(Taskspace t) {
	bool noError = true;

	if (t.x < 0.0) {
		printf("X-COORDINATE OUTSIDE WORKSPACE \n");
		noError = false;
	}

	if (t.x < 40.64 && abs(t.y) < 75) {
		//printf("X-COORDINATE OUTSIDE WORKSPACE \n");
		noError = false;
	}

	if (sqrt(t.x*t.x + t.y*t.y) <= BODY_RADIUS && t.z <= ROBOT_BASE_HEIGHT) {
		printf("ERROR: ATTEMPTING TO MOVE END EFFECTOR INSIDE BODY RADIUS \n");
		noError = false;
	}

	if (t.z < 0.0) {
		printf("Z-COORDINATE OUTSIDE WORKSPACE \n");
		noError = false;
	}

	return noError;

}



int Microbot::SetDelta(Registerspace start, Registerspace finish) {
	for (int i = 1; i <= 8; i++) {
		delta.r[i] = finish.r[i] - start.r[i];
	};



	/*for (int i = 1; i <= 6; i++) {
		cout << "Finish " << finish.r[i] << " Start " << start.r[i] << endl;
	}
	cout << endl;
	*/

	return 1;

}

int Microbot::SetDelta(Jointspace start, Jointspace finish) {
	for (int i = 1; i <= 6; i++) {
		deltaJoints.j[i] = finish.j[i] - start.j[i];
	};



	/*for (int i = 1; i <= 6; i++) {
		cout << "Finish " << finish.r[i] << " Start " << start.r[i] << endl;
	}
	cout << endl;
	*/

	return 1;
}



int  Microbot::MoveTo(Taskspace &t) {
	Pose tmp = currentPose;


	int check1 = CheckWorkspaceLimits(t);
	if (check1 <= 0) {
		return 0;
	};

	int check2 = InverseKinematics(t, tmp.js);
	if (check2 <= 0) {
		return 0;
	}


	SetDelta(currentPose.js, tmp.js);

	JointToRegister(deltaJoints, delta);

	SpaceConvertion(tmp, t);

	SendStep(microbot_speed, delta);

	lastPose = currentPose;
	currentPose = tmp;

	return 1;

}

void Microbot::GoHome() {

	SetDelta(currentPose.js, home.js);
	JointToRegister(deltaJoints, delta);
	SendStep(microbot_speed, delta);

	lastPose = currentPose;
	currentPose = home;

};

void Microbot::CurrentPosition(Taskspace &t) {
	t = currentPose.ts;
};


int Microbot::SpaceConvertion(Pose &pose, Taskspace t) {


	Pose tmp = pose;
	tmp.ts = t;

	int check1 = CheckWorkspaceLimits(t);
	if (check1 <= 0) {
		return 0;
	};

	int check2 = InverseKinematics(t, tmp.js);
	if (check2 <= 0) {
		return 0;
	}

	JointToRegister(tmp.js, tmp.rs);

	pose = tmp;

	return 1;

}

int Microbot::SpaceConvertion(Pose &pose, Registerspace r) {

	Pose tmp = pose;
	tmp.rs = r;
	RegisterToJoint(tmp.rs, tmp.js);
	ForwardKinematics(tmp.js, tmp.ts);

	int check1 = CheckWorkspaceLimits(tmp.ts);
	if (check1 <= 0) {
		return 0;
	};
	int check2 = CheckWorkspaceLimits(tmp.js);
	if (check2 <= 0) {
		return 0;
	};

	pose = tmp;

	return 1;
}

int Microbot::SpaceConvertion(Pose &pose, Jointspace j) {

	Pose tmp = pose;
	tmp.js = j;
	JointToRegister(tmp.js, tmp.rs);
	ForwardKinematics(tmp.js, tmp.ts);

	int check1 = CheckWorkspaceLimits(tmp.ts);
	if (check1 <= 0) {
		return 0;
	};
	int check2 = CheckWorkspaceLimits(tmp.js);
	if (check2 <= 0) {
		return 0;
	};

	pose = tmp;

	return 0;
}

//#################### TOWER OF HANOI HANDLING ####################################

int Microbot::PickandPlace(Taskspace start, Taskspace finish, double height, double gripForce) {

	int extraHeight = 75;
	int talestTower = 25 * 6;
	int gripper = 0;

	Pose tmpGripHandler;
	Pose tmp;

	tmp.ts = start;


	//move above location 1 pre pickup

	tmp.ts.z = height;
	MoveTo(tmp.ts);


	//move to location 1

	tmp.ts = start;

	MoveTo(tmp.ts);

	//close gripper

	SendReset();

	printf("\n Closing Gripper");

	SendClose(235, gripForce);

	SendRead(tmpGripHandler.rs);
	SpaceConvertion(tmpGripHandler, tmpGripHandler.rs);

	gripper = tmp.ts.g + tmpGripHandler.ts.g;


	cout << "Gripper: " << tmp.ts.g << " + " << tmpGripHandler.ts.g << " = " << gripper << endl;

	//move above location 1 post pickup

	tmp.ts.z = height;
	tmp.ts.g = gripper;
	MoveTo(tmp.ts);

	//move above location 2 pre placement
	tmp.ts = finish;
	tmp.ts.z = height;
	tmp.ts.g = gripper;

	MoveTo(tmp.ts);

	//move to location 2
	tmp.ts = finish;
	tmp.ts.g = gripper;
	MoveTo(tmp.ts);

	//open gripper
	tmp.ts.g = tmp.ts.g + 30;
	MoveTo(tmp.ts);

	//move to location 2 post placement
	//tmp.ts = finish;
	tmp.ts.z = height;
	MoveTo(tmp.ts);

	return 1;
}


//This function fills in the position of the cubes and returns how many cubes were measure
int Microbot::MeasureCubes(Cube c[])//
{
	Taskspace t = currentPose.ts;
	Pose tmpGripHandler;
	int i = 0, l;
	Cube tmp;

	while (true)
	{
		//Gripper above cube, along x-axis, open 80mm
		t.z = 40;
		t.r = 90;
		t.g = 80;
		MoveTo(t);

		//Move gripper down to cube
		t.z = 10;
		MoveTo(t);

		//Use close command to measure width of cube
		SendReset();
		SendClose(microbot_speed, -1);
		SendRead(tmpGripHandler.rs);
		SpaceConvertion(tmpGripHandler, tmpGripHandler.rs);
		t.g += tmpGripHandler.ts.g;
		cout << "Gripper: " << t.g << " [mm]" << endl;
		SpaceConvertion(currentPose, t);
		tmpGripHandler.ts.g = t.g;
		MoveTo(t);

		//Open gripper back to 80mm
		t.g = 80;
		MoveTo(t);

		//Move 30mm above base
		t.z = 40;
		MoveTo(t);

		//Here we check whether a cube is present or not, and if it is done measuring all cubes
		for (int j = 1; j <= 5; j++)
		{
			//Find out which of the five cubes is being measured
			if (((CUBE[j] - 4) < tmpGripHandler.ts.g) && (tmpGripHandler.ts.g < (CUBE[j] + 4)))
			{
				c[i + 1].ts = t;
				c[i + 1].n = j;
				c[i++ + 1].size = CUBE[j];
				cout << "Cube assigned!\n";
				break;
			}
			//If the size didn't fit for any of the five cubes and is bigger than 20mm
			//or if it was the first measurement and didn't fit for any of the five cubes
			else if ((j == 5) && (tmpGripHandler.ts.g >= 20 || i == 0))
			{
				cout << "Unknown size or no cube measured\n";
				return i;
			}
			//
			else if (j == 5)
			{
				cout << "All cubes measured\n";
				for (int k = 1; k <= i; k++) {
					tmp = c[k];
					l = k;
					while (l > 1 && c[l - 1].n > tmp.n) {
						c[l] = c[l - 1];
						l--;
					}
					c[l] = tmp;
				}
				return i;
			}
		}
		//If there the gripper measured one of the five cubes it moves 50 mm to the side
		t.y += 65;
		MoveTo(t);
	}
	return i;
}

// Gleðileg jól