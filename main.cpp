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
	double num;
	int n;
	int m = 1;
	char starter;

	cout << "Press any key to start sorting and solving the Tower of Hanoi\n";
	cin >> starter;
	robot.SendClose(235, -1);
	robot.SendReset();
	//robot.UserInterface();
	
	next = { 125,-150,40,-90,0,0 };
	robot.MoveTo(next);
	next = { 275,-150,40,-90,90,0 };
	robot.MoveTo(next);


	tower[1].ts = { 175,-100,5,-90,90,0 };
	tower[2].ts = { 175,0,5,-90,90,0 };
	tower[3].ts = { 175,100,5,-90,90,0 };

	tower[1].height = 0;
	
	n = robot.MeasureCubes(cube);
	


	printf("Moving cubes to Tower 1:\n");
	if (n > 0) 
	{
		robot.SortCubes(cube, tower[1], n);
	}
	num = (pow(2, n) - 1);
	printf("\nIt will take %g moves to solve the Tower of Hanoi with %d cubes\n",num , n);

	robot.TowerofHanoi(n, 1, 2, 3, m, cube, tower,num);
	next = { 150,100,150,-90,90,0 };
	robot.MoveTo(next);
	next = { 125,0,150,-90,90,0 };
	robot.MoveTo(next);
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