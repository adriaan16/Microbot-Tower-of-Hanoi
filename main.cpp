#include "kinematics.h"

using namespace std;

int main()
{
	Microbot robot;				// Local variable of the microbot class
	Taskspace next;			// Local variable for input of motor steps
	int spe = 235;				// Motor speed; should not be higher than 240
	Cube cube[6];
	Tower tower[4];
	double num;
	int n;
	int m = 1;
	char starter;

	tower[1].ts = { 175,-100,5,-90,90,0 };
	tower[1].height = 0;
	tower[2].ts = { 175,0,5,-90,90,0 };
	tower[3].ts = { 175,100,5,-90,90,0 };


	cout << "Press any key to start sorting and solving the Tower of Hanoi\n";
	cin >> starter;
	robot.SendClose(235, -1);
	robot.SendReset();
	
	next = { 125,0,40,-90,0,0 };
	robot.MoveTo(next);
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


}