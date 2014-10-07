/* �w�b�_ */
#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<time.h>
#include<stdint.h>
#include<setjmp.h>  //��Ǐ��I�W�����v�֐�

/* �}�N����` */
#define MOVE_VELO 100                 //�ԗ։�]���x[mm/sec]
#define TURN_VELO -100                //�ԗ։�]���x[mm/sec]
#define WHEEL_DISTANCE 30         //��̎ԗ֊Ԃ̋���[cm]
#define CICLE_TIME_SEC 0.1             //�T�C�N���^�C��[sec]
//#define CICLE_TIME 1000             //�T�C�N���^�C��[msec] (�������Windows�̏ꍇ)
#define CICLE_TIME 100000          //�T�C�N���^�C��[usec] (�������Ubuntu�̏ꍇ)
#define TARGET_X 1000                 //x���ւ̖ڕW�ړ�����[mm]
#define TARGET_Y 300                   //y���ւ̖ڕW�ړ�����[mm]
#define TURN_CLOCKWISE_ANGLE 80             //�ڕW��]�p�x(���v���)
#define TURN_COUNTERCLOCKWISE_ANGLE 100     //�ڕW��]�p�x(�����v���)
#define TURN_TARGET_ANGLE 90
#define SCAN_CLOCKWISE_ANGLE 180                   //�X�L�����p�x(���v���)
#define SCAN_COUNTERCLOCKWISE_ANGLE 90   //�X�L�����p�x(�����v���)
#define AVOID_SIDE 400                //�������ւ̈ړ�����(��Q�����)
#define AVOID_VIRTICAL 300        //�c�����ւ̈ړ�����(��Q�����)
#define SENSOR_THRESHOLD 415 //���̌��m�̕��̊ԋ�����臒l
#define BACK_DISTANCE 100
#define TRUE 1
#define FALSE -1
#define MAX 9999
#define MIN 0
#define CREATE_ACROSS 200 //irobot create�̔��a[mm]
#define JUDGE_NOISE 200 //�O��l�̊
#define JUDGE_CORNER_BUMP_NUM 5

#define CELL_NUM 40
#define CELL_SIZE_X 50
#define CELL_SIZE_Y 50
#define LINE_MAX 99999

/* �O�Ǖ`��p */
#define TARGET_ANGLE_5 5
#define MODIFY_5 2
#define SIMILAR_AREA_X 100
#define SIMILAR_AREA_Y 100 


/* �v���g�^�C�v�錾 */
void count_time();
double getRadian(int);
struct localization_data getRelativePos(struct localization_data, int, int, double);
struct localization_data initiate_pos();
void initiate_vel_omega();
void update_theta();
void getCurrentPos(int, int);
void turnCounterClockwise(int, int);
void turnClockwise(int, int);
void avoidObject();
void emergencyAvoidObjectVirticalWay();
void emergencyAvoidObjectSideWay();
void avoidSideWayIrregular(int);
void avoidVirticalWayIrregular(int);
void avoidVirticalWayRegular();
void goBack(int);
struct scanData getEdgeObject(struct scanData);
struct scanData scanClockwise(struct scanData);
struct scanData scanCounterClockwise();
struct objectPos_data getObjectPos(int, int);
int checkBumpPosDifference(struct objectPos_data, struct objectPos_data);
int checkCorner(int);

///////////////////////////////////////////////////
//             �\���̂̒�`                           //
//////////////////////////////////////////////////

/* ���Ȉʒu�c���̍\���� */
struct localization_data{
	int x;                    //x���W
	int y;                    //y���W
	int theta;              //�p���p�x
	int vel;                 //���x
	int omega;           //�p���x
};

/* ���̈ʒu�̍\���� */
struct objectPos_data{
	int x;
	int y;
};

struct scanData{
	int min_x;
	int min_y;
	int max_x;
	int max_y;
};


///////////////////////////////////////////////
//     �O���[�o���ϐ��̒�`                  //
//////////////////////////////////////////////
struct objectPos_data objectPos;
struct localization_data sum_data;
char *fname1 = "create_position.dat";
FILE *fp1;
char *fname2 = "object_position.txt";
FILE *fp2;
char *fname3 = "zyuusin_position.txt";
FILE *fp3;
time_t operate_start, operate_time;
jmp_buf ma;           //������ۑ�

////////////////////////////////////////////
//         �ėp���W���[��                    //
////////////////////////////////////////////

/* ���W�A���ϊ� */
double getRadian(int degree){
	double deg = degree;
	return deg * M_PI / 180;
}

void robotStop(){
	directDrive(0, 0);
}


/////////////////////////////////////////////
//           �����擾���W���[��             //
/////////////////////////////////////////////

/* �T�C�N���^�C���̃J�E���g */
void count_time(){
	int cicle_time = CICLE_TIME;
	clock_t start, end;
	start = clock();
	while(1){
		end = clock();
		if((end - start) >= cicle_time)
			break;
	}
}

//////////////////////////////////////////////////
//     �Z���T�f�[�^�擾���W���[��             //
/////////////////////////////////////////////////

/* �Z���T����A�i���O�l�擾->����[mm]�֕ϊ��C�o�� */
int getSensor(){
	uint8_t buf[2];
	uint16_t val;
	int sensor_value;
	readRawSensor(SENSOR_ANALOG_SIGNAL, buf, sizeof(buf));
	val = buf[0] << 8|buf[1];
	sensor_value = val;
	//printf("analog = %d distance = %lf\n", sensor_value, (0.5271*sensor_value-0.9939)*25.4); 
	//return sensor_value;
	return (0.5271 * sensor_value - 0.9939) * 25.4 + 15;
}

//////////////////////////////////////////////////
//         ���������W���[��                        //
/////////////////////////////////////////////////

/* x,y���W�C�p���p�x�C���x�C�p���x�̏����� */
struct localization_data initiate_pos(){
	struct localization_data initiate;
	
	initiate.x = 0;
	initiate.y = 0;
	initiate.theta = 0;
	initiate.vel = 0;
	initiate.omega = 0;
	
	return initiate;
}

/* ���x�C�p���x�̏����� */
void initiate_vel_omega(){
	sum_data.vel = 0;
	sum_data.omega = 0;
}

/* �p�x>=360�Ȃ�΁C�p�xmod360��Ԃ��֐� */
void update_theta(){
	if(sum_data.theta >= 360){
		sum_data.theta = sum_data.theta % 360;
	}
}

//////////////////////////////////////////////////
//         ���Ȉʒu���胂�W���[��               //
/////////////////////////////////////////////////

/* �O��̈ʒu�Ƃ̑��Έʒu�̎Z�o */
struct localization_data getRelativePos(struct localization_data past, int vel_right, int vel_left,double cicle_time){
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

/* ���Έʒu����Ɏ��Ȉʒu�̍X�V */
void getCurrentPos(int vel_left, int vel_right){
	struct localization_data relative_data;
	double cicle_time_sec = CICLE_TIME_SEC;
	
	relative_data = 
	getRelativePos(sum_data, vel_right, vel_left, cicle_time_sec);
	sum_data.x += relative_data.x;
	sum_data.y += relative_data.y;
	sum_data.theta += relative_data.theta;
	sum_data.vel = relative_data.vel;
	sum_data.omega = relative_data.omega;
}


void goBack(int target_distance){
	int vel_left = TURN_VELO;
	int vel_right = TURN_VELO;
	int temp_x = sum_data.x;
	int temp_y = sum_data.y;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(abs(sum_data.x - temp_x) >= target_distance || abs(sum_data.y - temp_y) >= target_distance){
			directDrive(0, 0);
			break;
		}
		else{
			count_time();
		}
	}
	waitTime(1);
}
		
void robotBack(int target_distance){
	int vel_left = TURN_VELO;
	int vel_right = TURN_VELO;
	int temp_x = sum_data.x;
	int temp_y = sum_data.y;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp1){
		while(1){
			getCurrentPos(vel_left, vel_right);
			printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			fprintf(fp1, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.x - temp_x) >= target_distance || abs(sum_data.y - temp_y) >= target_distance){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	waitTime(1);
}	


struct objectPos_data getObjectPos(int sensor_distance, int create_theta){
	double create_rad = getRadian(create_theta);
	struct objectPos_data unitObjectPos;
	unitObjectPos.x = sum_data.x + (sensor_distance * cos(create_rad));
	unitObjectPos.y = sum_data.y + (sensor_distance * sin(create_rad));
	return unitObjectPos;
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


/* �����v���ɐ���(�X�L��������) */
void turnCounterClockwise(int targetAngle, int modify_value){

	int vel_left = TURN_VELO;
	int vel_right = MOVE_VELO;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int operate_flag = -1;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		totalAngle += getAngle();
		if(abs(totalAngle - tempAngle) >= targetAngle - modify_value){
			directDrive(0, 0);
			waitTime(1);
			break;
		}
	}
	sum_data.theta += targetAngle;
}

/* ���v���ɐ���(�X�L��������) */
void turnClockwise(int targetAngle, int modify_value){
	int vel_left = MOVE_VELO;
	int vel_right = TURN_VELO;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int operate_flag = -1;
	directDrive(vel_left, vel_right);
	while(1){
		totalAngle += getAngle();
		if(abs(totalAngle - tempAngle) >= targetAngle - modify_value){
			directDrive(0, 0);
			waitTime(1);
			break;
		}
	}
	sum_data.theta -= targetAngle;
}

/* Bump�������̍��W���擾 */
struct objectPos_data getBumpPos(){
	struct objectPos_data BumpPos;
	BumpPos.x = sum_data.x;
	BumpPos.y = sum_data.y;
	return BumpPos;
}

int checkBumpPosDifference(struct objectPos_data currentBumpPos, struct objectPos_data beforeBumpPos){
	struct objectPos_data bumpPosDifference;
	bumpPosDifference.x = currentBumpPos.x - beforeBumpPos.x;
	bumpPosDifference.y = currentBumpPos.y - beforeBumpPos.y;
	if(abs(bumpPosDifference.x) < SIMILAR_AREA_X && abs(bumpPosDifference.y) < SIMILAR_AREA_Y)
		return TRUE;
	else
		FALSE;
}

int checkCorner(int count){
	if(count == JUDGE_CORNER_BUMP_NUM)
		return TRUE;
	else
		return FALSE;
}

/* �O�Ǖ`��p���W���[�� */
void serchOutEdge(){
	struct objectPos_data currentBumpPos;
	struct objectPos_data beforeBumpPos;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag = 0;
	int corner_flag;
	int count_corner = 0;
	int count  = 0;
	int trueOrFalse;
	beforeBumpPos.x = beforeBumpPos.y = currentBumpPos.x = currentBumpPos.y = 0;
	directDrive(vel_left, vel_right);
	if(fp1){
		while(1){
			getCurrentPos(vel_left, vel_right);
			//printf("(%d, %d) : deg = %d\n", sum_data.x, sum_data.y, sum_data.theta);
			fprintf(fp1, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			bump_flag = getBumpsAndWheelDrops();
			//printf("bump_flag = %d\n", bump_flag);
			if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
				currentBumpPos = getBumpPos();
				trueOrFalse = checkBumpPosDifference(currentBumpPos, beforeBumpPos);
				if(trueOrFalse == TRUE){
					count++;
				}
				else{
					count = 0;
				}
				corner_flag = checkCorner(count);
				if(corner_flag == TRUE)
					count_corner++;
				printf("count_corner = %d\n", count_corner);
				if(sum_data.y < 200 && count_corner == 3)
					break;
				robotStop();
				waitTime(1);
				robotBack(BACK_DISTANCE);
				robotStop();
				waitTime(1);
				turnCounterClockwise(TARGET_ANGLE_5, MODIFY_5);
				directDrive(vel_left, vel_right);
				beforeBumpPos = currentBumpPos;
			}
			count_time();
		}
	}
	robotStop();
}




int main(void){
	fp1 = fopen(fname1, "wt");
	fp2 = fopen(fname2, "wt");
	fp3 = fopen(fname3, "wt");
	startOI_MT("/dev/ttyUSB0");
	sum_data = initiate_pos();
	serchOutEdge();


	stopOI_MT();
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	//printf("before\n");
	//getMap();
	//printf("after\n");
	
	return 0;
}

