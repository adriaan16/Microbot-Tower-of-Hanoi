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
	/*
	robot.SetSpeed(spe);
	robot.CurrentPosition(current);


	while (GUI){
		robot.CurrentPosition(next);
		printf("Input the coordinates (X [mm], Y [mm], Z [mm], P [deg], R [deg], G [mm]): ");

			getline(cin, input);
			buffer << input;
			buffer >> next.x >> next.y >> next.z >> next.p >> next.r >> next.g;
			std::stringstream().swap(buffer);

		printf("Going from (%g mm, %g mm, %g mm, %g deg, %g deg, %g mm) ", current.x,current.y,current.z,current.p,current.r,current.g);
		printf("to (%g mm, %g mm, %g mm, %g deg, %g deg, %g mm)\n", next.x, next.y, next.z, next.p, next.r, next.g);

			robot.MoveTo(next);

			robot.CurrentPosition(current);

		printf("Do you want to continue? (1/0): ");
			cin >> GUI;
			cin.ignore();

		while ((GUI != 0) && (GUI != 1))
		{
			printf("Invalid input\n Do you want to continue? (1/0): ");
			cin >> GUI;
			cin.ignore();
		}
	}
	robot.GoHome();

	return 0;

	*/


	//adding test comment


	Taskspace start;
	Taskspace finish;

	start.x = 200;
	start.y = 75;
	start.z = 55;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = -75;
	finish.z = 5;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = 75;
	start.z = 32;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = 0;
	finish.z = 5;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = -75;
	start.z = 5;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = 0;
	finish.z = 35;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = 75;
	start.z = 5;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = -75;
	finish.z = 5;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = 0;
	start.z = 32;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = 75;
	finish.z = 5;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = 0;
	start.z = 5;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = -75;
	finish.z = 35;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);

	start.x = 200;
	start.y = 75;
	start.z = 5;
	start.p = -90;
	start.r = 0;
	start.g = 70;

	finish.x = 200;
	finish.y = -75;
	finish.z = 60;
	finish.p = -90;
	finish.r = 0;
	finish.g = 70;

	robot.PickandPlace(start, finish, 100, -1);


	robot.GoHome();

}