/* �w�b�_ */
#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<time.h>
#include<stdint.h>

/* �}�N����` */
#define MOVE_VELO 100                 //�ԗ։�]���x[mm/sec]
#define TURN_VELO -100                //�ԗ։�]���x[mm/sec]
#define WHEEL_DISTANCE 30         //��̎ԗ֊Ԃ̋���[cm]
#define CICLE_TIME_SEC 0.1             //�T�C�N���^�C��[sec]
//#define CICLE_TIME 1000             //�T�C�N���^�C��[msec] (�������Windows�̏ꍇ)
#define CICLE_TIME 100000          //�T�C�N���^�C��[usec] (�������Ubuntu�̏ꍇ)
#define OPERATE_FINISH 600         //�����I������[sec]
#define FIELD_SIDE_LENGTH 3300  //�t�B�[���h����(y������)�̒���[mm]
#define FIELD_VIRTICAL_LENGTH 4700 //�t�B�[���h�c��(x������)�̒���[mm]
#define SMALL_FIELD_ANGLE 60 //���_�p�x(��������)
#define BIG_FIELD_ANGLE 120   //���_�p�x(�傫����)
#define TARGET_X 1000                 //x���ւ̖ڕW�ړ�����[mm]
#define TARGET_Y 300                   //y���ւ̖ڕW�ړ�����[mm]
#define TURN_CLOCKWISE_ANGLE 80             //�ڕW��]�p�x(���v���)
#define TURN_COUNTERCLOCKWISE_ANGLE 100     //�ڕW��]�p�x(�����v���)
#define TURN_TARGET_ANGLE 90
#define SCAN_CLOCKWISE_ANGLE 90                   //�X�L�����p�x(���v���)
#define SCAN_COUNTERCLOCKWISE_ANGLE 180   //�X�L�����p�x(�����v���)
#define AVOID_SIDE 400                //�������ւ̈ړ�����(��Q�����)
#define AVOID_VIRTICAL 600        //�c�����ւ̈ړ�����(��Q�����)
#define SENSOR_THRESHOLD 300 //���̌��m�̕��̊ԋ�����臒l
#define BACK_DISTANCE 50
#define TRUE 1
#define FALSE -1
#define MAX 9999
#define MIN 0
#define CREATE_ACROSS 250 //irobot create�̔��a[mm]
#define JUDGE_NOISE 200 //�O��l�̊
/* ���M�n����̌덷�l */
#define MODIFY_CLOCKWISE_180 7
#define MODIFY_CLOCKWISE_120 10
#define MODIFY_CLOCKWISE_90 12
#define MODIFY_CLOCKWISE_60 7
#define MODIFY_COUNTERCLOCKWISE_120 6
#define MODIFY_COUNTERCLOCKWISE_90 8//scanCounterclock
#define MODIFY_COUNTERCLOCKWISE_60 7

#define AVOID_SIDE_BAIRITU 0.5

//TURN_COUNTERCLOCKWISE_ANGLE, MODIFY_COUNTERCLOCKWISE_120
//TURN_CLOCKWISE_ANGLE, MODIFY_COUNTERCLOCKWISE_60
//TURN_CLOCKWISE_ANGLE, MODIFY_CLOCKWISE_60
//TURN_COUNTERCLOCKWISE_ANGLE, MODIFY_CLOCKWISE_120


#define CELL_NUM 40
#define CELL_SIZE_X 50
#define CELL_SIZE_Y 50
#define LINE_MAX 99999

/* �O�Ǖ`��p */
#define TARGET_ANGLE_5 5
#define MODIFY_5 2
#define SIMILAR_AREA_X 100
#define SIMILAR_AREA_Y 100 
#define WAIT_TIME 0.5
#define JUDGE_CORNER_BUMP_NUM 5

/* �v���g�^�C�v�錾 */
int check_operate_time();
void count_time();
double getRadian(int);
struct localization_data getRelativePos(struct localization_data, int, int, double);
struct localization_data initiate_pos();
void initiate_vel_omega();
void update_theta();
void getCurrentPos(int, int);
void goVirticalWay();
void goSideWay();
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
struct scanData scanClockwise();
struct scanData scanCounterClockwise(struct scanData);
struct objectPos_data getObjectPos(int, int);
int checkBumpPosDifference(struct objectPos_data, struct objectPos_data);
int checkCorner(int);
void setCornerAngle(int);
void setRobotAngle(int);
struct objectPos_data getCornerPos(int);
void setFieldLength(struct objectPos_data, int);

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

struct draw_data{
	int min_x;
	int min_y;
	int max_x;
	int max_y;
	int flag;
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
char *fname1 = "../../share/draw_field_form.txt";
FILE *fp1;
char *fname2 = "../../share/object_position.txt";
FILE *fp2;
char *fname3 = "../../share/zyuusin_position.txt";
FILE *fp3;
char *fname4 = "../../share/not_exist_object_position.txt";
FILE *fp4;
time_t operate_start, operate_time;
int firstCornerAngle;
int secondCornerAngle;
int thirdCornerAngle;
int fourthCornerAngle;
int robotAngle[5];
int virticalFieldLength;
int sideFieldLength;
int robot_theta_flag;

////////////////////////////////////////////
//         �ėp���W���[��                    //
////////////////////////////////////////////

void writeRobotPos(){
	if(fp4){
		fprintf(fp4, "%4d, %4d\n", sum_data.x - 100, sum_data.y - 100);
		fprintf(fp4, "%4d, %4d\n", sum_data.x - 100, sum_data.y);
		fprintf(fp4, "%4d, %4d\n", sum_data.x - 100, sum_data.y + 100);
		fprintf(fp4, "%4d, %4d\n", sum_data.x, sum_data.y -100);
		fprintf(fp4, "%4d, %4d\n", sum_data.x, sum_data.y);
		fprintf(fp4, "%4d, %4d\n", sum_data.x, sum_data.y + 100);
		fprintf(fp4, "%4d, %4d\n", sum_data.x + 100, sum_data.y - 100);
		fprintf(fp4, "%4d, %4d\n", sum_data.x + 100, sum_data.y);
		fprintf(fp4, "%4d, %4d\n", sum_data.x + 100, sum_data.y + 100);
	}
}

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

/* ���{�b�g���쎞�Ԃ̎擾 */
int check_operate_time(){
	static int operate_finish = OPERATE_FINISH;
	operate_time = time(NULL);
	printf("operate time = %d\n", (int)(operate_time - operate_start));
	if((int)(operate_time - operate_start) >= operate_finish)
		return TRUE;
	else
		return FALSE;
}

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

///////////////////////////////////////////////////
//                  �̂��̂�����                    //
///////////////////////////////////////////////////
void song(){
	uint8_t song[2];
	song[0] = 55;
	song[1] = 32;
	writeSong(0, 2, song);
	playSong(0);
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

/////////////////////////////////////////////////
//     ��Q���̃X�L����/������W���[��     //
/////////////////////////////////////////////////

/* ��Q���X�L����/���S�̍\�����W���[�� */
void avoidObject(){
	struct scanData scanPos;
	int firstPos_y = sum_data.y;
	int target_position;
	int flag;
	int zyuusin_x;
	int zyuusin_y;
	directDrive(0, 0);
	waitTime(0.5);
	/* CREATE�̎p����Ԃ̔���	*/
	if(abs(sum_data.theta % 360) < 90){
		robot_theta_flag = TRUE;
	}else{
		robot_theta_flag = FALSE;
	}
	/* 1.�X�L����(�[�̎擾) */
	scanPos = scanClockwise();
	scanPos = scanCounterClockwise(scanPos);
	if(fp3){
		zyuusin_x = (scanPos.min_x + scanPos.max_x) / 2;
		zyuusin_y = (scanPos.min_y + scanPos.max_y) / 2;
		fprintf(fp3, "center point = %d, %d\n", zyuusin_x, zyuusin_y);
	}
	if(robot_theta_flag == TRUE){
		target_position = scanPos.max_y;
	}else{
		target_position = scanPos.min_y;
	}
	printf("target_position = %d\n", target_position);
	/* 2.�������ɉ��(�����͕s��) */
	avoidSideWayIrregular(target_position);
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	/* 3.�c�����ɉ��(�����͈��) */
	avoidVirticalWayRegular();
	/* 4.���̂̐��ʂ֕����]���{�X�L����(�[�̎擾) */
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	scanPos = scanClockwise();
	scanPos = scanCounterClockwise(scanPos);
	if(robot_theta_flag == TRUE){
		target_position = scanPos.max_x;
	}else{
		target_position = scanPos.min_x;
	}
	/* 5.�c�����ɉ��(�����͕s��) */
	printf("start a\n");
	printf("targetPosition = %d\n", target_position);
	avoidVirticalWayIrregular(target_position);
	printf("finish a\n");
	
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	/* 6.�������ɉ��(���̍��W�܂�) */
	avoidSideWayIrregular(firstPos_y);
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}

void emergencyAvoidObjectVirticalWay(){
	struct scanData scanPos;
	int target_position;
	waitTime(0.5);
	goBack(BACK_DISTANCE);
	scanPos = scanClockwise();
	scanPos = scanCounterClockwise(scanPos);
	if(sum_data.theta % 360 == 0){
		target_position = scanPos.max_x;
	}
	else{
		target_position = scanPos.min_x;
	}
	avoidVirticalWayIrregular(target_position);
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}

void emergencyAvoidObjectSideWay(){
	printf("start emergency\n");
	struct scanData scanPos;
	int target_position;
	waitTime(0.5);
	goBack(BACK_DISTANCE);
	scanPos = scanClockwise();
	scanPos = scanCounterClockwise(scanPos);
	if(sum_data.theta % 360 == -90 || sum_data.theta % 360 == 270){
		target_position = scanPos.min_y;
	}
	else{
		target_position = scanPos.max_y;
	}
	printf("start\n");
	avoidSideWayIrregular(target_position);
	printf("finish\n");
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}



/* ���̂̒[�̍��W�擾 */
struct scanData getEdgeObject(struct scanData scanPos){
	if(scanPos.min_x > objectPos.x)
		scanPos.min_x = objectPos.x;
	if(scanPos.max_x < objectPos.x)
		scanPos.max_x = objectPos.x;
	if(scanPos.min_y > objectPos.y)
		scanPos.min_y = objectPos.y;
	if(scanPos.max_y < objectPos.y)
		scanPos.max_y = objectPos.y;
	
	return scanPos;
}

/* ���v���֐���E���̂��X�L���� */
struct scanData scanClockwise(){
	struct scanData scanPos;
	int vel_left = 100;
	int vel_right = -100;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int targetAngle = SCAN_CLOCKWISE_ANGLE;
	int sensor_distance;
	int draw_count = 0;
	int getAngle_value;
//	int beforeAngle = sum_data.theta;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp2){
		while(1){
			getAngle_value = getAngle();
			totalAngle += getAngle_value;
			sensor_distance = getSensor();
			//if(draw_count % 5 == 0)
			//fprintf(fp2, "totalAngle = %d : getAngle = %d\n", totalAngle, getAngle_value);
			printf("totalAngle = %d : getAngle = %d\n", totalAngle, getAngle_value);
			if(sensor_distance < 1.2 * SENSOR_THRESHOLD){
				objectPos = getObjectPos(sensor_distance, sum_data.theta + (totalAngle - tempAngle));
				//printf("x = %d, y = %d, theta = %d\n", objectPos.x, objectPos.y, sum_data.theta - (totalAngle - tempAngle));
				//printf("%d = %d + (%d - %d)\n", sum_data.theta + (totalAngle - tempAngle), sum_data.theta, totalAngle, tempAngle);
				//if(objectPos.y > 0){ //y�������̃X�L�����͈͎w��
					//if(objectPos.x > -1.73 * objectPos.y && objectPos.x < -1.73 * objectPos.y + FIELD_VIRTICAL_LENGTH){ //x�������̃X�L�����͈͎w��
						//printf("robot = (%d, %d), object = (%d, %d)\n", sum_data.x, sum_data.y, objectPos.x, objectPos.y);
						//if(draw_count % 5 == 0)
							fprintf(fp2, "%4d, %4d\n", objectPos.x, objectPos.y);
						if(objectPos.y > 350){
							scanPos = getEdgeObject(scanPos);
						}
					//}
				//}
			}
			if(abs(totalAngle - tempAngle) >= targetAngle - MODIFY_CLOCKWISE_180){
				directDrive(0, 0);
				printf("totalAngle=%d, tempAngle=%d\n", totalAngle, tempAngle);
				waitTime(1);
				break;
			}
			draw_count++;
			count_time();
		}
	}
	sum_data.theta -= targetAngle;
	printf("min=(%d, %d), max = (%d, %d)\n", scanPos.min_x, scanPos.min_y, scanPos.max_x, scanPos.max_y);
	return scanPos;
}	

/* �����v���֐���E���̂��X�L���� */
struct scanData scanCounterClockwise(struct scanData scanPos){
	scanPos.min_x = scanPos.min_y = MAX;
	scanPos.max_x = scanPos.max_y = MIN;
	int vel_left = -100;
	int vel_right = 100;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int targetAngle = SCAN_COUNTERCLOCKWISE_ANGLE;
	int sensor_distance;
	int draw_count = 0;
	int getAngle_value = 0;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp2){
		while(1){
			getAngle_value = getAngle();
			totalAngle += getAngle_value;
			sensor_distance = getSensor();
			//if(draw_count % 5 == 0)
			//fprintf(fp2, "totalAngle = %d : getAngle = %d\n", totalAngle, getAngle_value);
			printf("totalAngle = %d : getAngle = %d\n", totalAngle, getAngle_value);
			if(sensor_distance < 1.2*SENSOR_THRESHOLD){
				objectPos = getObjectPos(sensor_distance, sum_data.theta + (totalAngle - tempAngle));
						//printf("robot = (%d, %d), object = (%d, %d)\n", sum_data.x, sum_data.y, objectPos.x, objectPos.y);
						//if(draw_count % 5 == 0){
							fprintf(fp2, "%4d, %4d\n", objectPos.x, objectPos.y);
						//}
						if(objectPos.y > 350){
							scanPos = getEdgeObject(scanPos);
						}
					//}
				//}
			}
			if(abs(totalAngle - tempAngle) >= targetAngle - MODIFY_COUNTERCLOCKWISE_90){
				directDrive(0, 0);
				printf("totalAngle = %d, tempAngle = %d\n", totalAngle, tempAngle);
				waitTime(0.5);
				break;
			}
			draw_count++;
			count_time();
		}
	}
	sum_data.theta += targetAngle;
	return scanPos;
}

/* ��(y��)�����։�𓮍�(�����͔C��) */
void avoidSideWayIrregular(int edge_of_object){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	int draw_count = 0;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}
		bump_flag = getBumpsAndWheelDrops();
		if(bump_flag ==1 || bump_flag == 2 || bump_flag == 3){
			emergencyAvoidObjectVirticalWay();
			directDrive(vel_left, vel_right);
		}
		if(sum_data.theta % 360 == 90 || sum_data.theta % 360 == -270){
			if(sum_data.y > edge_of_object + CREATE_ACROSS){
				printf("%d > %d - %d\n", sum_data.y, edge_of_object, CREATE_ACROSS);
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}else{
			if(sum_data.y < edge_of_object - CREATE_ACROSS){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
				draw_count++;
			}
		}
	}
	waitTime(0.5);
}

/* �c(x��)�����։�𓮍�(�����͔C��) */
void avoidVirticalWayIrregular(int edge_of_object){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	int draw_count = 0;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}
		bump_flag = getBumpsAndWheelDrops();
		if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
			emergencyAvoidObjectSideWay();
			directDrive(vel_left, vel_right);
		}
		if(sum_data.theta %360 == 0){
			if(sum_data.x > edge_of_object + CREATE_ACROSS){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
		else{
			if(sum_data.x < edge_of_object - CREATE_ACROSS){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
				draw_count++;
			}
		}
	}
	waitTime(0.5);
}

/* �c(x��)�����։�𓮍�(�����͈��) */
void avoidVirticalWayRegular(){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	int temp_x = sum_data.x;
	int target_distance = AVOID_VIRTICAL;
	int draw_count = 0;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}		
		bump_flag = getBumpsAndWheelDrops();
		/* bump�����Ȃ�΋ً}��𓮍샂�[�h�� */
		if(bump_flag >= 1 && bump_flag <= 3){
			emergencyAvoidObjectSideWay();
			directDrive(vel_left, vel_right);
		}
		/* ��苗���i�񂾂Ȃ�ΏI�� */
		if(abs(sum_data.x - temp_x) >= target_distance){
			directDrive(0, 0);
			break;
		}
		else{
			count_time();
			draw_count++;
		}
	}
	waitTime(0.5);
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
	waitTime(0.5);
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
			//printf(fp1, "%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(abs(sum_data.x - temp_x) >= target_distance || abs(sum_data.y - temp_y) >= target_distance){
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}
	}
	waitTime(0.2);
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


void firstGoVirticalWay(){
	int target_distance = virticalFieldLength * 0.5;
	int firstStage_distance = target_distance * 0.7;
	int sensorThreshold = SENSOR_THRESHOLD;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int temp_x = sum_data.x;
	int sensor_distance;
	int before_sensor_distance = MAX;
	int totalDistance;
	int bump_flag = 0;
	int draw_count = 0;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}
		totalDistance = sum_data.x - temp_x;
		sensor_distance = getSensor();
		bump_flag = getBumpsAndWheelDrops();
		if(abs(totalDistance) < firstStage_distance){
			if(bump_flag == 1 || bump_flag == 3){
				robotStop();
				waitTime(0.2);
				robotBack(BACK_DISTANCE);
				robotStop();
				turnCounterClockwise(TARGET_ANGLE_5, MODIFY_5);
				sum_data.theta = 0;
				directDrive(vel_left, vel_right);
			}
			else if(bump_flag == 2){
				avoidObject();
				directDrive(vel_left, vel_right);
			}
		}
		else if(abs(totalDistance) < target_distance){
			if(sensor_distance <= SENSOR_THRESHOLD - 50){
				if((before_sensor_distance - sensor_distance) < JUDGE_NOISE){
					avoidObject();
					directDrive(vel_left, vel_right);
				}
			}
			else if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
				avoidObject();
				directDrive(vel_left, vel_right);
			}
		}
		else if(abs(totalDistance) >= target_distance){
			if(sensor_distance <= SENSOR_THRESHOLD){
				if((before_sensor_distance - sensor_distance) < JUDGE_NOISE){
					printf("sensor_distance = %d\n", sensor_distance);
					robotStop();
					waitTime(0.5);
					break;
				}
			}
		}
		count_time();
		before_sensor_distance = sensor_distance;
		draw_count++;
	}
}


void goVirticalWay(){
	int target_distance = virticalFieldLength * 0.45;
	int sensorThreshold = SENSOR_THRESHOLD;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int temp_x = sum_data.x;
	int sensor_distance;
	int before_sensor_distance = MAX;
	int totalDistance;
	int bump_flag = 0;
	int draw_count = 0;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}
		totalDistance = sum_data.x - temp_x;
		sensor_distance = getSensor();
		bump_flag = getBumpsAndWheelDrops();
		if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
			waitTime(0.5);
			robotBack(BACK_DISTANCE);
			goto OUT;
		}
		else if(sensor_distance <= SENSOR_THRESHOLD){  //�Z���T�[�擾�l��臒l��
			if((before_sensor_distance - sensor_distance) < JUDGE_NOISE){
				if(abs(totalDistance) <= target_distance){
					OUT:
					//song();
					if(abs(sum_data.theta) % 360 < 90){
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 300, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 300, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 300, sum_data.y+100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 200, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 200, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x + 200, sum_data.y+100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x+100, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x+100, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x+100, sum_data.y+100);
					}else{
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 300, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 300, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 300, sum_data.y+100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 200, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 200, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x - 200, sum_data.y+100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x-100, sum_data.y-100);
						fprintf(fp4, "%4d, %4d\n", sum_data.x-100, sum_data.y);
						fprintf(fp4, "%4d, %4d\n", sum_data.x-100, sum_data.y+100);
					}
					avoidObject();
					directDrive(vel_left, vel_right);	
				}
				else{
					//song();
					directDrive(0, 0);
					waitTime(0.5);
					break;
				}
			}
		}
		count_time();
		before_sensor_distance = sensor_distance;
		draw_count++;
	}
}

void goSideWay(){
	int target_distance = 0.25 * sideFieldLength;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int temp_y = sum_data.y;
	int bump_flag = 0;
	int draw_count = 0;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(draw_count % 20 == 0){
			writeRobotPos();
		}
		bump_flag = getBumpsAndWheelDrops();
		if(abs(sum_data.y - temp_y) >= target_distance){
			directDrive(0, 0);
			waitTime(0.5);
			break;
		}
		else if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
			robotBack(BACK_DISTANCE);
			directDrive(0, 0);
			waitTime(0.5);
			break;
		}
		else{
			count_time();
			draw_count++;
		}
	}
}

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
			waitTime(0.2);
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
			waitTime(0.5);
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

void setRobotAngle(int count_corner){
	if(count_corner == 1){
		sum_data.theta = TARGET_ANGLE_5 * JUDGE_CORNER_BUMP_NUM;
	}else if(count_corner == 3){
		sum_data.theta = 180 + TARGET_ANGLE_5 * JUDGE_CORNER_BUMP_NUM;
	}
	robotAngle[count_corner] = sum_data.theta;
}

void setCornerAngle(int count_corner){
	if(count_corner == 2){
		firstCornerAngle = robotAngle[count_corner] - robotAngle[count_corner - 1];
	}
	else if(count_corner == 3){
		secondCornerAngle = robotAngle[count_corner] - robotAngle[count_corner - 1];
	}
	else{
		thirdCornerAngle = robotAngle[count_corner] - robotAngle[count_corner - 1];
		fourthCornerAngle = (360 - robotAngle[count_corner]) + TARGET_ANGLE_5 * JUDGE_CORNER_BUMP_NUM;
	}
}

/* �O�Ǖ`��p���W���[�� */
void serchOutEdge(){
	struct objectPos_data currentBumpPos;
	struct objectPos_data beforeBumpPos;
	struct objectPos_data firstCornerPos;
	struct objectPos_data secondCornerPos;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag = 0;
	int corner_flag;
	int count_corner = 0;
	int count  = 0;
	int draw_count = 0;
	int trueOrFalse;
	int diff_x;
	int diff_y;
	int modifyAngle;
	beforeBumpPos.x = beforeBumpPos.y = currentBumpPos.x = currentBumpPos.y = 0;
	directDrive(vel_left, vel_right);
	if(fp1){
		while(1){
			getCurrentPos(vel_left, vel_right);
			//printf("(%d, %d) : deg = %d\n", sum_data.x, sum_data.y, sum_data.theta);
			if(draw_count % 10 == 0){
				fprintf(fp1, "%4d, %4d\n", sum_data.x, sum_data.y);
			}
			bump_flag = getBumpsAndWheelDrops();
			//printf("bump_flag = %d\n", bump_flag);
			if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
				currentBumpPos = getBumpPos();   //bump�����ʒu���W�̎擾
				trueOrFalse = checkBumpPosDifference(currentBumpPos, beforeBumpPos); //�O��ƍ����bump�ʒu�̔�r
				if(trueOrFalse == TRUE){ //2�_�Ԃ���苗���ȓ��Ȃ��count++
					count++;
				}
				else{  //�����łȂ��Ȃ��count = 0
					count = 0;
				}
				/* count�����l�ɒB�����Ȃ�΋��Ɣ��f */
				corner_flag = checkCorner(count);
				if(corner_flag == TRUE){
					count_corner++;
					setRobotAngle(count_corner); // 1.���Ɣ��f���ꂽ���̃��{�b�g�p���p�x�̎擾
					if(count_corner == 1){
						firstCornerPos.x = sum_data.x;
						firstCornerPos.y = sum_data.y;
						virticalFieldLength = firstCornerPos.x;
					}
					else if(count_corner == 2){
						secondCornerPos.x = sum_data.x;
						secondCornerPos.y = sum_data.y;
						diff_x = secondCornerPos.x - firstCornerPos.x;
						diff_y = secondCornerPos.y - firstCornerPos.y;
						sideFieldLength = sqrt(pow(diff_x, 2) + pow(diff_y, 2));
					}
					if(count_corner > 1){
						setCornerAngle(count_corner); // 1.�Ŏ擾�����p���p�x����Ƀt�B�[���x�p�x�̎擾
					}
				}
				printf("count_corner = %d\n", count_corner);
				if(count_corner == 4){
					robotStop();
					waitTime(1);
					robotBack(BACK_DISTANCE);
					waitTime(1);
					turnCounterClockwise(360 - sum_data.theta, 10);
					break;
				}
				robotStop();
				waitTime(0.2);
				robotBack(BACK_DISTANCE);
				robotStop();
				turnCounterClockwise(TARGET_ANGLE_5, MODIFY_5);
				directDrive(vel_left, vel_right);
				beforeBumpPos = currentBumpPos;
			}
			count_time();
			draw_count++;
		}
	}
	robotStop();
}

/* �O�Ǖ`��p���W���[�� */

void serchDockingStationAndDocking(){
	int bump_flag = 0;
	directDrive(200, 200);
	int infrared;
	int chargingSource;
	int cnt = 0;
	while(1){
		bump_flag = getBumpsAndWheelDrops();
		infrared = readSensor(SENSOR_INFRARED);
		printf("%d\n", infrared);
		if(infrared < 255){
			cnt++;
		}else{
			cnt = 0;
		}
		if(cnt >=10){
			while(1){
				runCoverAndDockDemo();
				chargingSource = readSensor(SENSOR_CHARGING_SOURCES_AVAILABLE);
				if(chargingSource == 2){
					printf("Dockng success\n");
					goto END;
				}
			}			
		}
		else if(bump_flag == 1 || bump_flag == 2 || bump_flag == 3){
			robotStop();
			waitTime(0.2);
			robotBack(BACK_DISTANCE);
			robotStop();
			turnClockwise(5, MODIFY_5);
			directDrive(200, 200);
		}
		count_time();
	}
	END: ;
}

int main(void){
	int i;
	int turnCounterClockwiseAngle, turnClockwiseAngle;
	fp1 = fopen(fname1, "w");
	fp2 = fopen(fname2, "wt");
	fp3 = fopen(fname3, "wt");
	fp4 = fopen(fname4, "wt");
	startOI_MT("/dev/ttyUSB0");
	sum_data = initiate_pos();
	
	serchOutEdge();
	fclose(fp1);
	int temp_virtical = virticalFieldLength / 100;
	int temp_side = sideFieldLength / 100;
	double aspect_virtical = temp_virtical / temp_virtical + temp_side;
	double aspect_side = temp_side / temp_virtical + temp_side;
	if(fp3){
		fprintf(fp3, "\n-----------------------\n\n");
		fprintf(fp3, "firstCorner : %d\n", 180 - firstCornerAngle);
		fprintf(fp3, "secondCorner : %d\n", 180 - secondCornerAngle);
		fprintf(fp3, "thirdCorner : %d\n", 180 - thirdCornerAngle);
		fprintf(fp3, "fourthCorner : %d\n", 180 - fourthCornerAngle);
		fprintf(fp3, "\n-----------------------\n\n");
		fprintf(fp3, "virtical : %d,  side : %d\n", virticalFieldLength, sideFieldLength);
		fprintf(fp3, "\n ----------------------\n");
		//fprintf(fp3, "aspect ratio     %g : %g\n", aspect_virtical, aspect_side);
	}
	printf("firstCorner : %d\n", 180 - firstCornerAngle);
	printf("secondCorner : %d\n", 180 - secondCornerAngle);
	printf("thirdCorner : %d\n", 180 - thirdCornerAngle);
	printf("fourthCorner : %d\n", 180 - fourthCornerAngle);
	printf("virtical : %d,  side : %d\n", virticalFieldLength, sideFieldLength);
	turnCounterClockwiseAngle = (firstCornerAngle + thirdCornerAngle) / 2;
	turnClockwiseAngle = 180 - turnCounterClockwiseAngle;
	song();
	waitTime(3);

	sum_data.theta = 0;
	sum_data.x = 200;
	sum_data.y = 300;

/*
	virticalFieldLength = 500;
	sideFieldLength = 2800;
	goVirticalWay();
	//turnCounterClockwise(180, MODIFY_CLOCKWISE_180);
	turnCounterClockwise(90, 6);
	goSideWay();
	turnCounterClockwise(90, 6);
	goVirticalWay();
*/
/*	
	for(i=0;i<1;i++){
		goVirticalWay();
		turnCounterClockwise(turnCounterClockwiseAngle - 5, MODIFY_COUNTERCLOCKWISE_120);
		goSideWay();
		turnCounterClockwise(turnClockwiseAngle + 5, MODIFY_COUNTERCLOCKWISE_60);
		goVirticalWay();
		turnClockwise(turnClockwiseAngle + 5, MODIFY_CLOCKWISE_60);
		goSideWay();
		turnClockwise(turnCounterClockwiseAngle + 5, MODIFY_CLOCKWISE_120);
		update_theta();
	}
*/
//	virticalFieldLength = 2500;
//	sideFieldLength = 3000;
	
	
//	turnCounterClockwiseAngle = 110;
//	turnClockwiseAngle = 70;
	firstGoVirticalWay();
	turnCounterClockwise(turnCounterClockwiseAngle - 5, MODIFY_COUNTERCLOCKWISE_120);
	printf("goside\n");
	goSideWay();
	turnCounterClockwise(turnClockwiseAngle + 5, MODIFY_COUNTERCLOCKWISE_60);
	goVirticalWay();
	turnClockwise(turnClockwiseAngle + 5, MODIFY_CLOCKWISE_60);
	goSideWay();
	turnClockwise(turnCounterClockwiseAngle -5, MODIFY_CLOCKWISE_120);
	update_theta();
	goVirticalWay();
	fclose(fp2);
	fclose(fp3);
/*	struct scanData scanPos;
	scanPos = scanClockwise();
	scanPos = scanCounterClockwise(scanPos);
*/
	serchDockingStationAndDocking();
	stopOI_MT();

	fclose(fp4);
	//printf("before\n");
	//getMap();
	//printf("after\n");
	
	return 0;
}

