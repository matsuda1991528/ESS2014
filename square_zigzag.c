/* definition of header */
#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<time.h>

/* definition of macro */
#define MOVE_VELO 100
#define TURN_VELO -100
#define WHEEL_DISTANCE 30
#define CICLE_TIME_SEC 1
//#define CICLE_TIME 1000       //to simyurate
#define CICLE_TIME 1000000  //to move create
#define TARGET_X 1000
#define TARGET_Y 300
#define TARGET_ANGLE 90
#define TRUE 1
#define FALSE -1

int check_operate_time();
void count_time();
double getRadian(int);
struct localization_data getRelativePos(struct localization_data, int, int, int);
struct localization_data initiate_pos();
void initiate_vel_omega();
void update_theta();
void getCurrentPos(int, int);
int goVirticalWay();
int goSideWay();
int turnCounterClockewise();
int turnClockewise();


/* structure of create's position */
struct localization_data{
	int x;
	int y;
	int theta;
	int vel;
	int omega;
};

struct localization_data sum_data;
char *fname = "create_position.txt";
FILE *fp;
time_t operate_start, operate_time;



int check_operate_time(){
	static int operate_finish = 100;
	operate_time = time(NULL);
	printf("operate time = %d\n", (int)(operate_time - operate_start));
	if((int)(operate_time - operate_start) >= operate_finish)
		return TRUE;
	else
		return FALSE;
}

/* count cicle time */
void count_time(){
	int cicle_time = CICLE_TIME;
	//int cicle_time = 1000;
	clock_t start, end;
	start = clock();
	while(1){
		end = clock();
		if((end - start) >= cicle_time)
			break;
	}
}

/* angle -> radian */
double getRadian(int degree){
	double deg = degree;
	return deg * M_PI / 180;
}

/* soutaiiti no sansyutu */
struct localization_data getRelativePos(struct localization_data past, int vel_right, int vel_left,int cicle_time){
	struct localization_data relative;
	int angular_vel;
	int wheel_distance = WHEEL_DISTANCE;
	double past_theta_rad;
	
	relative.vel = (vel_right + vel_left) / 2;
	relative.omega = wheel_distance * (vel_right - vel_left) / 2;
	past_theta_rad = getRadian(past.theta);
	relative.x = past.vel * cos(past_theta_rad) * cicle_time;
	relative.y = past.vel * sin(past_theta_rad) * cicle_time;
	relative.theta = (past.omega * cicle_time) / 100;
	
	return relative;
}

struct localization_data initiate_pos(){
	struct localization_data initiate;
	
	initiate.x = 0;
	initiate.y = 0;
	initiate.theta = 0;
	initiate.vel = 0;
	initiate.omega = 0;
	
	return initiate;
}

void initiate_vel_omega(){
	sum_data.vel = 0;
	sum_data.omega = 0;
}

void update_theta(){
	if(sum_data.theta >= 360){
		sum_data.theta = sum_data.theta % 360;
	}
}

void getCurrentPos(int vel_left, int vel_right){
	struct localization_data relative_data;
	int cicle_time_sec = CICLE_TIME_SEC;
	
	relative_data = 
	getRelativePos(sum_data, vel_right, vel_left, cicle_time_sec);
	sum_data.x += relative_data.x;
	sum_data.y += relative_data.y;
	sum_data.theta += relative_data.theta;
	sum_data.vel = relative_data.vel;
	sum_data.omega = relative_data.omega;
}

/* algorithm to operate move zigzag */
/* if wall exist left */
// 1. goVerticalWay();
// 2. turnClockwise();
// 3. goSideWay();
// 4. turnClockWise();
// 5. goVerticalWay();
// 6. turnCounterClockWise();
// 7. goSideWay();
// 8. turnCounterClockWise();

/* else if wall exist right */
// turnClockWise() <--> turnCounterClockWise()

int goVirticalWay(){
	int target_distance = TARGET_X;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int operate_flag = -1;
	int temp_x = sum_data.x;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp){
		while(1){
			getCurrentPos(vel_left, vel_right);
			fprintf(fp, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.x - temp_x) >= target_distance){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	operate_flag = check_operate_time();
	return operate_flag;
}

int goSideWay(){
	int target_distance = TARGET_Y;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int operate_flag = -1;
	int temp_y = sum_data.y;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp){
		while(1){
			getCurrentPos(vel_left, vel_right);
			fprintf(fp, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.y - temp_y) >= target_distance){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	operate_flag = check_operate_time();
	return operate_flag;
}

int turnCounterClockwise(){
	int target_angle = TARGET_ANGLE;
	int vel_left = TURN_VELO;
	int vel_right = MOVE_VELO;
	int operate_flag = -1;
	int temp_theta = sum_data.theta;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp){
		getCurrentPos(vel_left, vel_right);
		while(1){
			getCurrentPos(vel_left, vel_right);
			fprintf(fp, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.theta - temp_theta) >= target_angle){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	operate_flag = check_operate_time();
	return operate_flag;
}

int turnClockwise(){
	int target_angle = TARGET_ANGLE;
	int vel_left = MOVE_VELO;
	int vel_right = TURN_VELO;
	int operate_flag = -1;
	int temp_theta = sum_data.theta;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp){
		getCurrentPos(vel_left, vel_right);
		while(1){
			getCurrentPos(vel_left, vel_right);
			fprintf(fp, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.theta - temp_theta) >= target_angle){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	operate_flag = check_operate_time();
	return operate_flag;
}


int main(void){
	int operate_flag = FALSE;
	fp = fopen(fname, "wt");
	startOI_MT("/dev/ttyUSB0");
	operate_start = time(NULL);
	check_operate_time();
	sum_data = initiate_pos();
	while(1){
		operate_flag = goVirticalWay();
		if(operate_flag == TRUE)
			break;
		operate_flag = turnCounterClockwise();
		if(operate_flag == TRUE)
			break;
		operate_flag = goSideWay();
		if(operate_flag == TRUE)
			break;
		operate_flag = turnCounterClockwise();
		if(operate_flag == TRUE)
			break;
		operate_flag = goVirticalWay();
		if(operate_flag == TRUE)
			break;
		operate_flag = turnClockwise();
		if(operate_flag == TRUE)
			break;
		operate_flag = goSideWay();
		if(operate_flag == TRUE)
			break;
		operate_flag = turnClockwise();
		if(operate_flag == TRUE)
			break;
		update_theta();
	}
	stopOI_MT();
	fclose(fp);
	
	return 0;
}	
