#include <stdio.h>

//application related
int choice;

void createMenu();
void printModelParameters();
void setModelParameters();
void listCommands();

//Delta robot parameters
float e = 115.0;     // end effector's side length
float f = 457.3;     // base's side length
float re = 232.0;	 // lower arm
float rf = 112.0;	 // upper arm

int main(int argc, char **argv)
{
	createMenu();
	
	return 0;
}

void createMenu() {
	printf("Delta Robot Kinematics Reference\n\n");
	printf("Choose the following settings:\n");
	printf("\t1. View current delta robot model parameters\n");
	printf("\t2. Change current delta robot model parameters\n");
	printf("\t3. Commands list\n");
	printf("\t4. Start\n\n");
	printf("Choice: ");
	scanf("%d", &choice);
	
	while(choice > 0 && choice < 5) {
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
				return;
				break;
		}
	}
}

void printModelParameters() {
	printf("Current model parameters:\n");
	printf("%-20s%-20s%-20s%-20s\n", "Base", "End effector", "Lower arm", "Upper arm");
	printf("%-20f%-20f%-20f%-20f\n", f, e, re, rf);
	printf("\n\n");
	createMenu();
}

void setModelParameters() {
	printf("New model parameters (use arrows to get to next/previous fields)\n");
	printf("%-20s%-20s%-20s%-20s\n", "Base", "End effector", "Lower arm", "Upper arm");
	
	printf("\n\n");
	createMenu();
}

void listCommands() {
	
	printf("\n\n");
	createMenu();
}