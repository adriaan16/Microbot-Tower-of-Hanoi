#include "kinematics.h"

using namespace std;

int main()
{
	Microbot robot;				// Local variable of the microbot class
	Taskspace next;
	Taskspace current;			// Local variable for input of motor steps
	int spe = 235;				// Motor speed; should not be higher than 240
	Cube cube[6];
	Tower tower[4];
	int n;
	int m = 1;
	
	
	robot.SendClose(235, -1);
	robot.SendReset();
	next = { 125,-125,50,-90,90,70 };
	robot.MoveTo(next);

	tower[1].ts = { 125,-75,5,-90,90,0 };
	tower[2].ts = { 125,0,5,-90,90,0 };
	tower[3].ts = { 125,75,5,-90,90,0 };

	tower[1].height = 0;


	n = robot.MeasureCubes(cube);
	if (n > 0) 
	{
		robot.SortCubes(cube, tower[1], n);
	}
	



	robot.TowerofHanoi(n, 1, 2, 3, m, cube, tower);




	robot.GoHome();
	/*
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
	*/
	//robot.UserInterface();
	//robot.GoHome();
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