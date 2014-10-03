#include<createoi.h>
#include<stdio.h>

#define MOVE_VELO 100
#define TURN_VELO -100
#define TARGET_ANGLE_5 5
#define MODIFY_5 0
#define TARGET_ANGLE_60 60
#define MODIFY_60 0

void turnCounterClockwise(int, int);

void serchOutEdge(){
	int bump_flag = 0;
	int count_corner = 0;
	while(1){
		directDrive(MOVE_VELO, MOVE_VELO);
		bump_flag = getBumpsAndWheelDrops();
		printf("bump_flag = %d\n", bump_flag);
		if(bump_flag == 1){
			directDrive(0, 0);
			waitTime(1);
			turnCounterClockwise(TARGET_ANGLE_5, MODIFY_5);
		}
		else if(bump_flag == 2 || bump_flag == 3){
			count_corner++;
			if(count_corner == 4)
				break;
			else{
				directDrive(0, 0);
				waitTime(1);
				turnCounterClockwise(TARGET_ANGLE_60, MODIFY_60);
			}
		}
	}
	directDrive(0, 0);
}

void turnCounterClockwise(int targetAngle, int modify_value){
	int currentAngle = getAngle();
	int pastAngle = currentAngle;
	directDrive(TURN_VELO, MOVE_VELO);
	while(1){
		currentAngle += getAngle();
		if(abs(currentAngle - pastAngle) >= targetAngle - modify_value){
			directDrive(0, 0);
			waitTime(1);
			break;
		}
	}
}

int main(void){
	startOI_MT("/dev/ttyUSB0");
	serchOutEdge();
	stopOI_MT();
	
	return 0;
}
