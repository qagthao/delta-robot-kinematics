/*
	Delta Robot Kinematics Reference - allow quick tests against TrossenRobotics' delta robot kinematics tutorial
	Copyright (C) 2014  Cong Nguyen

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	 * 
	 * Reference: http://forums.trossenrobotics.com/tutorials/introduction-129/delta-robot-kinematics-3276/
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

//application related
char command[256];

void createMenu();
void printModelParameters();
void setModelParameters();
void listCommands();
void process(char *);
void start();

//Delta robot parameters
float e = 115.0;     // end effector's side length
float f = 457.3;     // base's side length
float re = 232.0;	 // lower arm
float rf = 112.0;	 // upper arm

//TrossenRobotics' delta robot tutorial
float sqrt3;
float pi;    // PI
float sin120;
float cos120;
float tan60;
float sin30;
float tan30;

int delta_calcForward(float, float, float, float *, float *, float *);
int delta_calcAngleYZ(float, float, float, float *);
int delta_calcInverse(float, float, float, float *, float *, float *);

int main(int argc, char **argv)
{
	//initialize trigs floats
	// trigonometric constants
	sqrt3 = sqrt(3.0);
	pi = 3.141592653;    // PI
	sin120 = sqrt3/2.0;
	cos120 = -0.5;
	tan60 = sqrt3;
	sin30 = 0.5;
	tan30 = 1/sqrt3;

	printf("Delta Robot Kinematics Reference\n\n");
	createMenu();

	start();

	return 0;
}

/* DELTA ROBOT FUNCTIONS */

// inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
// returned status: 0=OK, -1=non-existing position
int delta_calcInverse(float x0, float y0, float z0, float *theta1, float *theta2, float *theta3)
{
	*theta1 = *theta2 = *theta3 = 0;
	int status = delta_calcAngleYZ(x0, y0, z0, theta1);
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);  // rotate coords to +120 deg
	if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);  // rotate coords to -120 deg
	return status;
}


// inverse kinematics
// helper functions, calculates angle theta1 (for YZ-pane)
int delta_calcAngleYZ(float x0, float y0, float z0, float *theta)
{
	float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
	y0 -= 0.5 * 0.57735    * e;    // shift center to edge
	// z = a + b*y
	float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
	float b = (y1-y0)/z0;
	// discriminant
	float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
	if (d < 0) return -1; // non-existing point
	float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
	float zj = a + b*yj;
	*theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
	return 0;
}

// forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
// returned status: 0=OK, -1=non-existing position
int delta_calcForward(float theta1, float theta2, float theta3, float *x0, float *y0, float *z0)
{
	float t = (f-e)*tan30/2;
	float dtr = pi/(float)180.0;

	theta1 *= dtr;
	theta2 *= dtr;
	theta3 *= dtr;

	float y1 = -(t + rf*cos(theta1));
	float z1 = -rf*sin(theta1);

	float y2 = (t + rf*cos(theta2))*sin30;
	float x2 = y2*tan60;
	float z2 = -rf*sin(theta2);

	float y3 = (t + rf*cos(theta3))*sin30;
	float x3 = -y3*tan60;
	float z3 = -rf*sin(theta3);

	float dnm = (y2-y1)*x3-(y3-y1)*x2;

	float w1 = y1*y1 + z1*z1;
	float w2 = x2*x2 + y2*y2 + z2*z2;
	float w3 = x3*x3 + y3*y3 + z3*z3;

	// x = (a1*z + b1)/dnm
	float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
	float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

	// y = (a2*z + b2)/dnm;
	float a2 = -(z2-z1)*x3+(z3-z1)*x2;
	float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

	// a*z^2 + b*z + c = 0
	float a = a1*a1 + a2*a2 + dnm*dnm;
	float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
	float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

	// discriminant
	float d = b*b - (float)4.0*a*c;
	if (d < 0) return -1; // non-existing point

	*z0 = -(float)0.5*(b+sqrt(d))/a;
	*x0 = (a1*(*z0) + b1)/dnm;
	*y0 = (a2*(*z0) + b2)/dnm;
	return 0;
}

/* USER INTERFACE FUNCTIONS */

void start() {
	while(1) {
		fseek(stdin,0,SEEK_END);
		printf(">> ");
		if(fgets(command, 256, stdin) != NULL) {
			process(command);
		}
	}
}

void process(char * instr)
{
	char *tokens;
	char params[4][256]; //create an array of strings
	float inputs[3];
	float outputs[3];

	int displayResultsFlag = 0;
	int doingInverse = 0;

	//split the strings into an array of strings
	int i = 0;
	tokens = strtok(instr, " ,\n");
	while(tokens != NULL) {
		strcpy(params[i++], tokens);
		tokens = strtok(NULL, " ,\n");
	}
	//process the array
	if(!strcmp(params[0], "-i")) {
		displayResultsFlag = 1;
		doingInverse = 1;
		for(i = 0; i < 3; i++) {
			inputs[i] = atof(params[i+1]);
		}
		delta_calcInverse(inputs[0], inputs[1], inputs[2], &outputs[0], &outputs[1], &outputs[2]);
	} else if(!strcmp(params[0], "-f")) {
		displayResultsFlag = 1;
		doingInverse = 0;
		for(i = 0; i < 3; i++) {
			inputs[i] = atof(params[i+1]);
		}
		delta_calcForward(inputs[0], inputs[1], inputs[2], &outputs[0], &outputs[1], &outputs[2]);
	} else if(!strcmp(params[0], "-m")) {
		createMenu();
	}
	
	if(displayResultsFlag) {
		displayResultsFlag = 0;
		if(doingInverse) {
			printf("Angles: %.3lf, %.3lf, %.3lf\n", outputs[0], outputs[1], outputs[2]);
		} else {
			printf("End effector coord.: %.3lf, %.3lf, %.3lf\n", outputs[0], outputs[1], outputs[2]);
		}
	}
}

void createMenu()
{
	int choice = 0;

	do {
		printf("\nChoose the following settings:\n");
		printf("\t1. View current delta robot model parameters\n");
		printf("\t2. Change current delta robot model parameters\n");
		printf("\t3. Commands list\n");
		printf("\t4. Start\n\n");
		printf("Choice: ");
		scanf("%d", &choice);
	} while(choice > 4 || choice < 0);

	switch(choice) {
	case 1:
		printf("\n\n");
		printModelParameters();
		break;
	case 2:
		printf("\n\n");
		setModelParameters();
		break;
	case 3:
		printf("\n\n");
		listCommands();
		break;
	case 4:
		printf("\n\n");
		start();
		break;
	}
}

void printModelParameters()
{
	printf("Current model parameters:\n");
	printf("%-20s%-20s%-20s%-20s\n", "Base", "End effector", "Lower arm", "Upper arm");
	printf("%-20f%-20f%-20f%-20f\n", f, e, re, rf);
	printf("\n\n");
	createMenu();
}

void setModelParameters()
{
	float input;
	printf("New model parameters (negative no. to return to menu):\n");
	
	printf("Base length: ");
	scanf("%f", &input);
	if(input >= 0) f = input;
	else printf("Base length unchanged\n");
	
	printf("End effector length: ");
	scanf("%f", &input);
	if(input >= 0) e = input;
	else printf("End effector length unchanged\n");
	
	printf("Upper arm length: ");
	scanf("%f", &input);
	if(input >= 0) rf = input;
	else printf("Upper arm length unchanged\n");

	printf("Lower arm length: ");
	scanf("%f", &input);
	if(input >= 0) re = input;
	else printf("Lower arm length unchanged\n");
	
	printf("\n");
	createMenu();
}

void listCommands()
{
	printf("Commands list:\n");
	printf("%-15s%-60s\n", " -i x, y, z", "calculate upper arm angles from end effector's coordinates");
	printf("%-15s%-60s\n", " -f a1, a2, a3", "calculate end effector's coordinates from upper arm angles");

	printf("\n\n");
	createMenu();
}