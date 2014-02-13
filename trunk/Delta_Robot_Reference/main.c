#include <stdio.h>
#include <string.h>

//application related
int choice;
char command[256];

void createMenu();
void printModelParameters();
void setModelParameters();
void listCommands();
void process(char *);

//Delta robot parameters
float e = 115.0;     // end effector's side length
float f = 457.3;     // base's side length
float re = 232.0;	 // lower arm
float rf = 112.0;	 // upper arm

int main(int argc, char **argv)
{
	createMenu();

	do {
		fseek(stdin,0,SEEK_END);
		printf(">> ");
		if(fgets(command, 256, stdin) != NULL) {
			process(command);
		}
	} while(!strcmp(command,"-q") && !strcmp(command,"-sq"));

	if(strcmp(command, "-sq")) {
		//save model parameters then let program end
	}
	
	return 0;
}

void process(char * instr)
{
	char *tokens;
	char params[4][256]; //create an array of strings
	int i = 0;

	//split the strings into an array of strings
	tokens = strtok(instr, " ,");
	while(tokens != NULL) {
		strcpy(params[i++], tokens);
		tokens = strtok(NULL, " ,");
	}

	//process the array
	if(!strcmp(params[0], "-i")) {
		printf("Inverse kinematics\n");
	} else if(!strcmp(params[0], "-f")) {
		printf("Forward kinematics\n");
	} else if(!strcmp(params[0], "-m")) {
		printf("\n\n");
		createMenu();
	}
}

void createMenu()
{
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
	printf("New model parameters:\n");
	printf("Base length: ");
	scanf("%f", &f);
	printf("End effector length: ");
	scanf("%f", &e);
	printf("Lower arm length: ");
	scanf("%f", &re);
	printf("Upper arm length: ");
	scanf("%f", &rf);

	printf("\n\n");
	createMenu();
}

void listCommands()
{
	printf("Commands list:\n");
	printf("%-15s%-60s\n", " -i x, y, z", "calculate upper arm angles from end effector's coordinates");
	printf("%-15s%-60s\n", " -f a1, a2, a3", "calculate end effector's coordinates from upper arm angles");
	printf("%-15s%-60s\n", " -m", "show menu");
	printf("%-15s%-60s\n", " -q", "quit without saving model parameters");
	printf("%-15s%-60s\n", " -sq", "save model parameters then quit");

	printf("\n\n");
	createMenu();
}
