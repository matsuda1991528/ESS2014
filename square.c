/* ヘッダ */
#include<createoi.h>
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<math.h>
#include<time.h>
#include<stdint.h>
#include<setjmp.h>  //非局所的ジャンプ関数

/* マクロ定義 */
#define MOVE_VELO 100                 //車輪回転速度[mm/sec]
#define TURN_VELO -100                //車輪回転速度[mm/sec]
#define WHEEL_DISTANCE 30         //二つの車輪間の距離[cm]
#define CICLE_TIME_SEC 1.0             //サイクルタイム[sec]
//#define CICLE_TIME 1000             //サイクルタイム[msec] (動作環境がWindowsの場合)
#define CICLE_TIME 1000000          //サイクルタイム[usec] (動作環境がUbuntuの場合)
#define OPERATE_FINISH 600         //走査終了時間[sec]
#define FIELD_SIZE_X 4700
#define FIELD_SIZE_Y 3300
#define FIELD_SIDE_LENGTH 3300  //フィールド横辺(y軸方向)の長さ[mm]
#define FIELD_VIRTICAL_LENGTH 4700 //フィールド縦辺(x軸方向)の長さ[mm]
#define SMALL_FIELD_ANGLE 60 //頂点角度(小さい方)
#define BIG_FIELD_ANGLE 120   //頂点角度(大きい方)
#define TARGET_X 1000                 //x軸への目標移動距離[mm]
#define TARGET_Y 300                   //y軸への目標移動距離[mm]
#define TURN_CLOCKWISE_ANGLE 80             //目標回転角度(時計廻り)
#define TURN_COUNTERCLOCKWISE_ANGLE 100     //目標回転角度(反時計廻り)
#define TURN_TARGET_ANGLE 90
#define SCAN_CLOCKWISE_ANGLE 180                   //スキャン角度(時計廻り)
#define SCAN_COUNTERCLOCKWISE_ANGLE 90   //スキャン角度(反時計廻り)
#define AVOID_SIDE 400                //横方向への移動距離(障害物回避)
#define AVOID_VIRTICAL 300        //縦方向への移動距離(障害物回避)
#define SENSOR_THRESHOLD 415 //物体検知の物体間距離の閾値
#define BACK_DISTANCE 200
#define TRUE 1
#define FALSE -1
#define PRA -1                              //setjmpの戻り値
#define MAX 9999
#define MIN 0
#define CREATE_ACROSS 200 //irobot createの半径[mm]
#define JUDGE_NOISE 200 //外れ値の基準
/* 超信地旋回の誤差値 */
#define MODIFY_CLOCKWISE_180 12
#define MODIFY_CLOCKWISE_120 6
#define MODIFY_CLOCKWISE_90 9
#define MODIFY_CLOCKWISE_60 8
#define MODIFY_COUNTERCLOCKWISE_120 6
#define MODIFY_COUNTERCLOCKWISE_90 9
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


/* プロトタイプ宣言 */
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
struct scanData scanClockwise(struct scanData);
struct scanData scanCounterClockwise();
struct objectPos_data getObjectPos(int, int);
void getMap();
void dockingOnly();
///////////////////////////////////////////////////
//             構造体の定義                           //
//////////////////////////////////////////////////

/* 自己位置把握の構造体 */
struct localization_data{
	int x;                    //x座標
	int y;                    //y座標
	int theta;              //姿勢角度
	int vel;                 //速度
	int omega;           //角速度
};

/* 物体位置の構造体 */
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
//     グローバル変数の定義                  //
//////////////////////////////////////////////
struct objectPos_data objectPos;
struct localization_data sum_data;
char *fname1 = "create_position.txt";
FILE *fp1;
//char *fname2 = "~/share/object_position.txt";
//char *fname2 = "././share/object_position.txt";
//char *fname2 = "/share/object_position.txt";
char *fname2 = "object_position.txt";
FILE *fp2;
char *fname3 = "zyuusin_position.txt";
FILE *fp3;
time_t operate_start, operate_time;
jmp_buf ma;           //環境情報を保存

////////////////////////////////////////////
//         汎用モジュール                    //
////////////////////////////////////////////

/* ラジアン変換 */
double getRadian(int degree){
	double deg = degree;
	return deg * M_PI / 180;
}


/////////////////////////////////////////////
//           時刻取得モジュール             //
/////////////////////////////////////////////

/* ロボット動作時間の取得 */
int check_operate_time(){
	static int operate_finish = OPERATE_FINISH;
	operate_time = time(NULL);
	printf("operate time = %d\n", (int)(operate_time - operate_start));
	if((int)(operate_time - operate_start) >= operate_finish)
		return TRUE;
	else
		return FALSE;
}

/* サイクルタイムのカウント */
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
//     センサデータ取得モジュール             //
/////////////////////////////////////////////////

/* センサからアナログ値取得->距離[mm]へ変換，出力 */
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
//         初期化モジュール                        //
/////////////////////////////////////////////////

/* x,y座標，姿勢角度，速度，角速度の初期化 */
struct localization_data initiate_pos(){
	struct localization_data initiate;
	
	initiate.x = 0;
	initiate.y = 0;
	initiate.theta = 0;
	initiate.vel = 0;
	initiate.omega = 0;
	
	return initiate;
}

/* 速度，角速度の初期化 */
void initiate_vel_omega(){
	sum_data.vel = 0;
	sum_data.omega = 0;
}

/* 角度>=360ならば，角度mod360を返す関数 */
void update_theta(){
	if(sum_data.theta >= 360){
		sum_data.theta = sum_data.theta % 360;
	}
}

//////////////////////////////////////////////////
//         自己位置推定モジュール               //
/////////////////////////////////////////////////

/* 前回の位置との相対位置の算出 */
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

/* 相対位置を基に自己位置の更新 */
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
//     障害物のスキャン/回避モジュール     //
/////////////////////////////////////////////////

/* 障害物スキャン/回避全体構成モジュール */
void avoidObject(){
	struct scanData scanPos;
	int firstPos_y = sum_data.y;
	int target_position;
	int flag;
	directDrive(0, 0);
	waitTime(1);
	/* CREATEの姿勢状態の判定	*/
	if(sum_data.theta == 0){
		flag = TRUE;
	}else{
		flag = FALSE;
	}
	/* 1.スキャン(端の取得) */
	scanPos = scanCounterClockwise();
	scanPos = scanClockwise(scanPos);
	if(flag == TRUE){
		target_position = scanPos.min_y;
	}else{
		target_position = scanPos.max_y;
	}
	printf("target_position = %d\n", target_position);
	/* 2.横方向に回避(距離は不定) */
	avoidSideWayIrregular(target_position);
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	/* 3.縦方向に回避(距離は一定) */
	avoidVirticalWayRegular();
	/* 4.物体の正面へ方向転換＋スキャン(端の取得) */
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	scanPos = scanCounterClockwise();
	scanPos = scanClockwise(scanPos);
	if(flag == TRUE){
		target_position = scanPos.max_x;
	}else{
		target_position = scanPos.min_x;
	}
	/* 5.縦方向に回避(距離は不定) */
	printf("start a\n");
	printf("targetPosition = %d\n", target_position);
	avoidVirticalWayIrregular(target_position);
	printf("finish a\n");
	
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_COUNTERCLOCKWISE_90);
	/* 6.横方向に回避(元の座標まで) */
	avoidSideWayIrregular(firstPos_y);
	turnClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}

void emergencyAvoidObjectVirticalWay(){
	struct scanData scanPos;
	int target_position;
	waitTime(1);
	goBack(BACK_DISTANCE);
	scanPos = scanCounterClockwise();
	scanPos = scanClockwise(scanPos);
	if(sum_data.theta == 0){
		target_position = scanPos.max_x;
	}
	else{
		target_position = scanPos.min_x;
	}
	avoidVirticalWayIrregular(target_position);
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}

void emergencyAvoidObjectSideWay(){
	printf("start emergency\n");
	struct scanData scanPos;
	int target_position;
	waitTime(1);
	goBack(BACK_DISTANCE);
	scanPos = scanCounterClockwise();
	scanPos = scanClockwise(scanPos);
	if(sum_data.theta == -90){
		target_position = scanPos.min_y;
	}
	else{
		target_position = scanPos.max_y;
	}
	printf("start\n");
	avoidSideWayIrregular(target_position);
	printf("finish\n");
	turnCounterClockwise(TURN_TARGET_ANGLE, MODIFY_CLOCKWISE_90);
}



/* 物体の端の座標取得 */
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

/* 時計廻りへ旋回・物体をスキャン */
struct scanData scanClockwise(struct scanData scanPos){
	int vel_left = MOVE_VELO;
	int vel_right = TURN_VELO;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int targetAngle = SCAN_CLOCKWISE_ANGLE;
	int sensor_distance;
//	int beforeAngle = sum_data.theta;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp2){
		while(1){
			totalAngle += getAngle();
			sensor_distance = getSensor();
			if(sensor_distance < 1.2 * SENSOR_THRESHOLD){
				objectPos = getObjectPos(sensor_distance, sum_data.theta + (totalAngle - tempAngle));
				//printf("x = %d, y = %d, theta = %d\n", objectPos.x, objectPos.y, sum_data.theta - (totalAngle - tempAngle));
				//printf("%d = %d + (%d - %d)\n", sum_data.theta + (totalAngle - tempAngle), sum_data.theta, totalAngle, tempAngle);
				//if(objectPos.y > 0){ //y軸方向のスキャン範囲指定
					//if(objectPos.x > -1.73 * objectPos.y && objectPos.x < -1.73 * objectPos.y + FIELD_VIRTICAL_LENGTH){ //x軸方向のスキャン範囲指定
						printf("robot = (%d, %d), object = (%d, %d)\n", sum_data.x, sum_data.y, objectPos.x, objectPos.y);
						fprintf(fp2, "%4d, %4d\n", objectPos.x, objectPos.y);
						scanPos = getEdgeObject(scanPos);
					//}
				//}
			}
			if(abs(totalAngle - tempAngle) >= targetAngle - MODIFY_CLOCKWISE_180){
				directDrive(0, 0);
				printf("totalAngle=%d, tempAngle=%d\n", totalAngle, tempAngle);
				waitTime(1);
				break;
			}
		}
	}
	sum_data.theta -= targetAngle;
	printf("min=(%d, %d), max = (%d, %d)\n", scanPos.min_x, scanPos.min_y, scanPos.max_x, scanPos.max_y);
	return scanPos;
}	

/* 反時計廻りへ旋回・物体をスキャン */
struct scanData scanCounterClockwise(){
	struct scanData scanPos;
	scanPos.min_x = scanPos.min_y = MAX;
	scanPos.max_x = scanPos.max_y = MIN;
	int vel_left = TURN_VELO;
	int vel_right = MOVE_VELO;
	int totalAngle = getAngle();
	int tempAngle = totalAngle;
	int targetAngle = SCAN_COUNTERCLOCKWISE_ANGLE;
	int sensor_distance;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	if(fp2){
		while(1){
			totalAngle += getAngle();
			sensor_distance = getSensor();
			if(sensor_distance < 1.2*SENSOR_THRESHOLD){
				objectPos = getObjectPos(sensor_distance, sum_data.theta + (totalAngle - tempAngle));
				//printf("x = %d, y = %d, theta = %d\n", objectPos.x, objectPos.y, sum_data.theta + (totalAngle - tempAngle));
				//printf("%d = %d + (%d - %d)\n", sum_data.theta + (totalAngle - tempAngle), sum_data.theta, totalAngle, tempAngle);
				//if(objectPos.y > 0){ //y軸方向のスキャン範囲指定
					//if(objectPos.x > -1.73 * objectPos.y && objectPos.x < -1.73 * objectPos.y + FIELD_VIRTICAL_LENGTH){ //x軸方向のスキャン範囲指定
						printf("robot = (%d, %d), object = (%d, %d)\n", sum_data.x, sum_data.y, objectPos.x, objectPos.y);
						fprintf(fp2, "%4d, %4d\n", objectPos.x, objectPos.y);
						scanPos = getEdgeObject(scanPos);
					//}
				//}
			}
			if(abs(totalAngle - tempAngle) >= targetAngle - MODIFY_COUNTERCLOCKWISE_90){
				directDrive(0, 0);
				printf("totalAngle = %d, tempAngle = %d\n", totalAngle, tempAngle);
				waitTime(1);
				break;
			}
		}
	}
	sum_data.theta += targetAngle;
	return scanPos;
}

/* 横(y軸)方向へ回避動作(距離は任意) */
void avoidSideWayIrregular(int edge_of_object){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		bump_flag = getBumpsAndWheelDrops();
		if(bump_flag >=1 && bump_flag <=3){
			emergencyAvoidObjectVirticalWay();
			directDrive(vel_left, vel_right);
		}
		if(sum_data.theta % 360 == 270 || sum_data.theta == -90){
			if(sum_data.y < edge_of_object - CREATE_ACROSS){
				printf("%d < %d - %d\n", sum_data.y, edge_of_object, CREATE_ACROSS);
				directDrive(0, 0);
				break;
			}
			else{
				count_time();
			}
		}else{
			if(sum_data.y > edge_of_object + CREATE_ACROSS){
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

/* 縦(x軸)方向へ回避動作(距離は任意) */
void avoidVirticalWayIrregular(int edge_of_object){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		bump_flag = getBumpsAndWheelDrops();
		if(bump_flag >=1 && bump_flag <=3){
			emergencyAvoidObjectSideWay();
			directDrive(vel_left, vel_right);
		}
		if(sum_data.theta == 0){
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
			}
		}
	}
	waitTime(1);
}

/* 縦(x軸)方向へ回避動作(距離は一定) */
void avoidVirticalWayRegular(){
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int bump_flag;
	int temp_x = sum_data.x;
	int target_distance = AVOID_VIRTICAL;
	
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		bump_flag = getBumpsAndWheelDrops();
		/* bumpしたならば緊急回避動作モードへ */
		if(bump_flag >= 1 && bump_flag <= 3){
			emergencyAvoidObjectSideWay();
			directDrive(vel_left, vel_right);
		}
		/* 一定距離進んだならば終了 */
		if(abs(sum_data.x - temp_x) >= target_distance){
			directDrive(0, 0);
			break;
		}
		else{
			count_time();
		}
	}
	waitTime(1);
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

void goVirticalWay(){
	int target_distance = TARGET_X;
	int sensorThreshold = SENSOR_THRESHOLD;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int operate_flag = -1;
	int temp_x = sum_data.x;
	int sensor_distance;
	int before_sensor_distance = MAX;
	int totalDistance;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("totalDistance = %d, %4d, %4d, %4d\n",totalDistance,  sum_data.x, sum_data.y, sum_data.theta);
		sensor_distance = getSensor();
		printf("before_sensor = %d, current_sensor %d\n", before_sensor_distance, sensor_distance);
		totalDistance = sum_data.x - temp_x;
		if(sensor_distance <= SENSOR_THRESHOLD){  //センサー取得値が閾値内
			if(abs(totalDistance) <= target_distance - 1 * SENSOR_THRESHOLD ){
				if((before_sensor_distance - sensor_distance) < JUDGE_NOISE){
					avoidObject();
					directDrive(vel_left, vel_right);	
				}
			}
		}
		if(abs(sum_data.x - temp_x) >= target_distance){
			directDrive(0, 0);
			waitTime(1);
			break;
		}
		else{
			count_time();
		}
		before_sensor_distance = sensor_distance;
	}
}

void goSideWay(){
	int target_distance = TARGET_Y;
	int vel_left = MOVE_VELO;
	int vel_right = MOVE_VELO;
	int operate_flag = -1;
	int temp_y = sum_data.y;
	initiate_vel_omega();
	directDrive(vel_left, vel_right);
	while(1){
		getCurrentPos(vel_left, vel_right);
		printf("%4d, %4d, %4d\n", sum_data.x, sum_data.y, sum_data.theta);
		if(abs(sum_data.y - temp_y) >= target_distance){
			directDrive(0, 0);
			waitTime(1);
			break;
		}
		else{
			count_time();
		}
	}
}

/* 反時計廻りに旋回(スキャン無し) */
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

/* 時計廻りに旋回(スキャン無し) */
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



int main(void){
	int key;
	int operate_flag = FALSE;
	int i;
	fp1 = fopen(fname1, "wt");
	fp2 = fopen(fname2, "wt");
	fp3 = fopen(fname3, "wt");
	startOI_MT("/dev/ttyUSB0");
	operate_start = time(NULL);
	check_operate_time();
	sum_data = initiate_pos();
	for(i=0;i<2;i++){
		goVirticalWay();
		turnCounterClockwise(TURN_COUNTERCLOCKWISE_ANGLE, MODIFY_COUNTERCLOCKWISE_120);
		goSideWay();
		turnCounterClockwise(TURN_CLOCKWISE_ANGLE, MODIFY_COUNTERCLOCKWISE_60);
		goVirticalWay();
		turnClockwise(TURN_CLOCKWISE_ANGLE, MODIFY_CLOCKWISE_60);
		goSideWay();
		turnClockwise(TURN_COUNTERCLOCKWISE_ANGLE, MODIFY_CLOCKWISE_120);
		update_theta();
	}
	directDrive(100, 100);
	int infrared;
	while(1){
		infrared = readSensor(SENSOR_INFRARED);
		if(infrared >= 252 && infrared < 255)
			dockingOnly();
	}
	//key = setjmp(ma);
	stopOI_MT();
	fclose(fp1);
	fclose(fp2);
	fclose(fp3);
	//printf("before\n");
	//getMap();
	//printf("after\n");
	
	return 0;
}

void dockingOnly(){
	int a;
	int chargingSource;
	
	while(1){
		a = runCoverAndDockDemo();
		chargingSource = readSensor(SENSOR_CHARGING_SOURCES_AVAILABLE);
		printf("charging source = %d\n",chargingSource);
		if(chargingSource == 2){
			printf("charge\n");
//			waitTime(3);
			break;
		}
	}
}