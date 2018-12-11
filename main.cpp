#include "kinematics.h"

using namespace std;

int main()
{
	Microbot robot;				// Local variable of the microbot class
	Registerspace delta;
	Taskspace next;
	Taskspace current;			// Local variable for input of motor steps
	int spe = 235;				// Motor speed; should not be higher than 240
	string input;
	stringstream buffer;
	int GUI = 1;
	Cube banani[5];
	Cube cube[6];
	Tower tower[4];

	int n = 5;
	int m = 1;
	
	/*
	tower[1].ts = { 200,-75,0,-90,90,0 };
	cube[1].ts = { 125,-75,0,-90,90,50 };
	cube[2].ts = { 125,0,0,-90,90,60 };
	cube[3].ts = { 125,75,0,-90,90,70 };
	robot.SortCubes(cube, tower[1], 3);
	*/

	robot.SendClose(235, -1);
	robot.SendReset();

	next = { 125,0,150,-90,0,0 } ;


	robot.MoveTo(next);

	cube[1].ts = { 200,-75,105,-90,90,50 };
	cube[2].ts = { 200,-75,80,-90,90,60 };
	cube[3].ts = { 200,-75,55,-90,90,70 };
	cube[4].ts = { 200,-75,30,-90,90,80 };
	cube[5].ts = { 200,-75,5,-90,90,90 };
	
	tower[1].ts = { 200,-75,130,-90,90,0 };
	tower[2].ts = { 200,0,5,-90,90,0 };
	tower[3].ts = { 200,75,5,-90,90,0 };

	robot.TowerofHanoi(n,1,2,3,m,cube,tower);

	//UserInterface(robot);

	/*
	robot.SendClose(spe, -1);
	robot.SendReset();
	next = {125,0,40,-90,90,0};
	robot.MoveTo(next);
	next = {175,-125,40,-90,90,0};
	robot.MoveTo(next);
	robot.MeasureCubes(banani);
	for (int i = 1; i <= 3; i++)
	{
		robot.PickandPlace();
	}
	robot.GoHome();
	*/

}