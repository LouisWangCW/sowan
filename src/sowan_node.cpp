#include <ros/ros.h>
#include <stdio.h>
#include <nav_msgs/GetMap.h>
#include <tf/tf.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <actionlib_msgs/GoalID.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>

#include "ether_server.hpp"
#include "Car_Setting_Pos.h"
#include "RS485_TECO_Motor.h"
#include "RouteSearch.h"

#include "sowan_node.h"
#include "GPIO.h"
#include "xmlRead.h"
#include "def.hpp"

#include <vector>
#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

#define MAIN_WORK_FREQUENCE 10

std::string Home_Root_String = HOME_ROOT;

UDP_SERVER EtherS;
MapRoute Map_Router;
PosTran TranFunc;
CarPosCalsulate RobotOdomInfo;
CarPosCalsulate::Position odometryPoint;
RaspberryGPIO GPIO_Control;
SOWAN_PARAMETER SOWAN(Home_Root_String +"/map/sowan.xml");
RS485_TECO_Motor::Motor_Control Motor("/dev/hl340");

//Move Need Para
cv::Point Start_P, End_P;
double g_vel = 0, t_vel = 0;
double x = 0, y = 0, z = 0, w = 0;
double couveRate = 1;
int ManualCouveRate = 5;
char lastMovCmd = 's';
int Motor_Step_L, Motor_Step_R;
double DegDiff_SHOW = 0, CircleRate_SHOW = 0;
int Motor_Pos_Read_M1=0, Motor_Pos_Read_M2=0;

//
int SystemWorkDetect = 0, SystemWorkDetect_Print = 0;
ros::Time Odom_current_time, Odom_last_time;
ros::Time Path_current_time, Path_last_time;
ecl::Mutex mutex;

//Voice
std::string VoiceP = Home_Root_String + "/Voice/Alarm04.wav";
bool StopOtherVoice = false;
ros::Time ProgramVoiceTime;
ros::Time VehicleStopVoiceTime;
int ProgramVoiceType;

//Debug Function Para
#define moveControlModeType 7
int baseControlMode = 0;
bool AutoMove = false;
bool normal_Mov_ShowFrist = true;;

//FPS Calculate Function Para
#define FPS_bufferSize 10
double FPS_buffer[FPS_bufferSize] = { 0 };
double FPS_avg = 0;
int FPS_buffer_count = 0;

//Path Move Function Para
#define Path_bufferSize 3
double Path_A_buffer[Path_bufferSize] = { 0 };
double Path_L_buffer[Path_bufferSize] = { 0 };
double Path_A_avg = 0;
double Path_L_avg = 0;
double Path_A_pre = 0;
double Path_L_pre = 0;
double Path_A_Now = 0;
double Path_L_Now = 0;
int Path_buffer_count = 0;
cv::Point2d golbalFinalPoint = cv::Point2d(0, 0);
std::vector<cv::Point> golbalPath;
std::vector<cv::Point> localPath;
#define movTag_bufferSize 5
cv::Point2d movTag_buffer[movTag_bufferSize] = { cv::Point2d(0,0) };
cv::Point2d localMovTag_avg = cv::Point2f(0, 0);
int movTag_buffer_count = 0;

//Patrol Mode Para
std::vector<cv::Point> PatrolPoint;
int patrolMovTagCount = 0;
int PatrolCycleCount = 1;
int PatrolCycleCount_Work = 1;
double PatrolCycleWatiTime = 0;
double PatrolCycleWatiTime_Work = 0;
bool PatrolMov_NextOverDeg_SW = true;
double PatrolOverDeg = 120;
double PatrolOverDeg_FinDeg = 45;

//Wait Mode Para
double WorkLoopTime = -1;
double WorkLoopSleepTime = 0;
double WorkLoopTime_Work = -1;
double WorkLoopSleepTime_Work = 0;
bool NowWorkLoop = true;
ros::Time WorkLoopTime_Ros;
bool WorkLoopSleepProcess = false;
double GetWorkLoopTime;
ros::Time WaitModetime_Pre;

// Charge Mode Para
std::vector<cv::Point> ChargePoint;
std::vector<cv::Point2d> Go_ChargePoint;
int GoChargePointCount = 0;
int CharegProcessSpd = 500;
int CharegProcessAcc = 300;
bool ChargeEndGoStart = false;
double ChargeWaitTime = 0;
ros::Time WaitChargeTime_Pre;
int GoChargePointDir = 0;

//Charge Point Move Para
std::vector<int> CamMotorCmdValue;
std::vector<int> CamMotorCmdType;
int CallCamWork = 0;
int TellCamMotorEnd = 0;
int CamMotorCmd = 0;
int CamSayCmd = 0;
int CamSayWorking = 0;
int CamMotorCmdDir = 0;
int CamMotorCmdDir_Pre = -1;

//Go Room Mode Para
std::vector<std::vector<cv::Point>> RoomSetPoint;
int GotoRoomCaseDir = 0;
int DoorOpenCounter = 0;
int SelRoomID = 0;
int SelRoomIDNext = -1;
std::vector<cv::Point2d> GoFrontDoorPoint;
int GoFrontDoorPointCount = 0;

//Go StartPos Mode Para
std::vector<cv::Point2d> GoStartPosPoint;
int GoStartPosPointCount = 0;
bool GetGoStartPos = false;

//normalMov Function Para
cv::Point2d normalMovTag = cv::Point2d(0, 0);
int normalMovHitObjCount = 0;
char normalMovHitObj_Dir = 's';
int normalMovHitObj_Value = 0;
int normalMovHitObj_Spd = 0;
int HitObjectRunawayType = 0;
int normalMovAdvCaseDir = 0;
double normalMovDegOffset = -3;

//OneMotionMov Function Para
int OneMotionMovDir = 0;
int OneMotionMovDir_Tag_1 = 0;
int OneMotionMovDir_Tag_2 = 0;

//BaseMov Function Para
geometry_msgs::PoseStamped BaseMovgoal;
int BaseMovTagCount = 0;
int BaseMovTagCaseDir = 0;
int BaseMovEsc = 0;
int GetGobalPathDir = 0;

//Mode Change System Para
int ModeNow = 0;
int ModeChangeCmd = 0;
bool getModeChangeCmd = false;

//Gui System Para
cv::Mat User_GUI;
cv::Mat Map_Monitor;
cv::Mat Para_Monitor;
cv::Mat Laser_Monitor;
cv::Mat Laser_Monitor_With_deg;
cv::Mat ALL_Monitor;
cv::Mat TEST_Monitor;
cv::Rect Map_Rect;
#define ButtonSizeW 100
#define ButtonSizeH 50
#define LaserWindowsSize 400
const int LaserWindowsLenth = 200;
const float LaserWindowsMeter = LaserWindowsLenth * 0.05;
int switchWindowsSel = -1;
int switchEtherSel = -1;
double Map2LaserRate = 1;

//init config
double FrontSafeDistance = 0.2 + 0.2;
double BackSafeDistance = 0.2 + 0.2;
double SideSafeDistance = 0.25;
double ArrivalRange = 0.1;
double MaxCircleSpd = 0.33;
double MaxLineSpd = 0.2;
double MaxTurnSpd = 0.2;
double MaxTurnRadius = (MaxLineSpd / MaxTurnSpd) - ArrivalRange;
double MinCircleNeedDeg = 5;
double MinTurnNeedDeg = 30;
double BackMovEscapeDistance = 0.1;
double DoorOpenDistance = 1.5;
double DoorOpenWidth = 0.4;
double DoorOpenWidth_half = 0.5 * DoorOpenWidth;
int normalMoveType = 0;

//init setting
double acc = 1500, spd = 500, turnRate = 0.2;
double FrontObjectClose_Value;
double FrontObjectClose_Value_Save;

//Type Bool
bool FrontObjectClose = false;
bool FrontObjectClose_L = false;
bool FrontObjectClose_R = false;
bool FrontObjectOpen_L = false;
bool FrontObjectOpen_R = false;
bool BackObjectClose = false;
bool BackObjectClose_L = false;
bool BackObjectClose_R = false;
bool DoorClosed = false;
bool FrontObject_LowOnly = false;
bool FrontObject_Laser = false;
bool LowSensorInstall = false;


//Mode Bool
bool PatrolProcessWorking = false;
bool GoStartPosWorking = false;
bool WaitProcessWorking = false;
bool RoomProcessWorking = false;
bool cam_Trac_Working = false;
bool GoChargePosWorking = false;


//Switch Bool
bool HitObjectStop = true;
bool goal_pub_switch = false;
bool goal_cancel_pub = false;
bool get_path_return = false;
#define UserGuiMode_type 3
int UserGuiMode = 1;
bool UserStopRoomProcess = false;
bool GoRoomByNormalMov = true;
bool GetGobalPathTest = false;


int main(int argc, char **argv) {
	printf("sowan main Start\n");
	SetSystemInitStep(SystemInitStep::fu_main);
	ros::init(argc, argv, "sowan");
	ros::NodeHandle n;

	ecl::Thread Node_Subscriber_Thread;
	ecl::Thread Node_Publisher_Thread;
	ecl::Thread ETHER_Thread;

	ETHER_Thread.start(ETHER_RECV);
	Node_Subscriber_Thread.start(Node_Subscriber);
	Node_Publisher_Thread.start(Node_Publisher);

	main_init();

	tf::TransformListener listener(ros::Duration(MAIN_WORK_FREQUENCE));

	ros::Rate loop_rate(MAIN_WORK_FREQUENCE);

	while (ros::ok())
	{
		ros::spinOnce();

		tf::StampedTransform transform;
		try {
			listener.lookupTransform("map", "base_link",
				ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}
		x = transform.getOrigin().x();
		y = transform.getOrigin().y();
		z = transform.getRotation().z();
		w = transform.getRotation().w();

		TranFunc.TF_Pos_update(x, y, w, z);

		main_function();

		loop_rate.sleep();
	}

	Node_Subscriber_Thread.join();
	Node_Publisher_Thread.join();
	ETHER_Thread.join();

	printf("ros End\n");

	return 0;

}
void main_init() {

	SowanParaSetting();

	Map_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	Para_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	Laser_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	ALL_Monitor = Mat::zeros(cv::Size(400 + 50, 800), CV_8UC3);

	TEST_Monitor = Mat::zeros(cv::Size(400, 400), CV_8UC3);

	cv::line(ALL_Monitor, cv::Point(400, 0), cv::Point(400, 800), cv::Scalar(255, 0, 255), 3);
	cv::Point ppp;
	cv::line(ALL_Monitor, cv::Point(400, 0), cv::Point(400 + 50, 0), cv::Scalar(255, 0, 255), 2);
	for (int y = 1; y <= RoomSetPoint.size(); y += 1) {
		ppp = cv::Point(400, y * 50);
		cv::line(ALL_Monitor, ppp, ppp + cv::Point(50, 0), cv::Scalar(255, 0, 255), 2);
		cv::putText(ALL_Monitor, std::to_string(y), ppp + cv::Point(3, 3), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 100, 255));
	}

	float switchScale = 0.85;
	float mapScale = 1.5;

	namedWindow("ALL_Monitor", 0);
	resizeWindow("ALL_Monitor", (400 + 50) * switchScale, 800 * switchScale);
	namedWindow("Map_Monitor", 0);
	resizeWindow("Map_Monitor", 400 * mapScale, 400 * mapScale);
	//setMouseCallback("Map_Monitor", mapOnMouse, NULL);
	setMouseCallback("ALL_Monitor", switchOnMouse, NULL);

	Motor.Motor_Pos_Set(1, 0);
	Motor.Motor_Pos_Set(2, 0);

	RobotOdomInfo.initial(Motor.Motor_Pos_Read(1), Motor.Motor_Pos_Read(2), 0, 0, 0);

	Motor.Motor_Error_Limit();

	Odom_current_time = ros::Time::now();
	Odom_last_time = Odom_current_time;

	Path_last_time = ros::Time::now();
	WorkLoopTime_Ros = ros::Time::now();

	BaseMovgoal.pose.position.z = 0;
	BaseMovgoal.pose.orientation.x = 0;
	BaseMovgoal.pose.orientation.y = 0;

	User_GUI = imread(Home_Root_String +"/src/ButtonBase.png");

	ModeNow = SystemMode::StandbyMode;
	SetSystemInitStep(SystemInitStep::fu_main_init);

	if (SOWAN.AutoMotorOn) {
		Motor.ServoOn(1);
		Motor.ServoOn(2);
	}
	else {
		Motor.ServoOFF(1);
		Motor.ServoOFF(2);
	}


}
void SowanParaSetting() {

	FrontSafeDistance = SOWAN.FrontSafeDistance;
	BackSafeDistance = SOWAN.BackSafeDistance;
	SideSafeDistance = SOWAN.SideSafeDistance;
	printf("Front SafeDistance =%.2f,back =%.2f, side =%.2f\n", FrontSafeDistance, BackSafeDistance, SideSafeDistance);

	ArrivalRange = SOWAN.ArrivalRange;
	DoorOpenDistance = SOWAN.DoorOpenDistance;
	DoorOpenWidth = SOWAN.DoorOpenWidth;
	DoorOpenWidth_half = 0.5 * DoorOpenWidth;

	printf("ArrivalRange =%.2f,DoorOpenDistance =%.2f\n", ArrivalRange, DoorOpenDistance);


	MaxCircleSpd = SOWAN.MaxCircleSpd;
	MaxLineSpd = SOWAN.MaxLineSpd;
	MaxTurnSpd = SOWAN.MaxTurnSpd;
	MaxTurnRadius = (MaxLineSpd / MaxTurnSpd) - ArrivalRange;
	printf("MaxCircleSpd =%f,MaxLineSpd =%f,MaxTurnSpd =%f\n", MaxCircleSpd, MaxLineSpd, MaxTurnSpd);

	normalMoveType = SOWAN.normalMoveType;


	MinCircleNeedDeg = SOWAN.MinCircleNeedDeg;
	MinTurnNeedDeg = SOWAN.MinTurnNeedDeg;
	printf("MinCircleNeedDeg =%f,MinTurnNeedDeg =%f\n", MinCircleNeedDeg, MinTurnNeedDeg);

	normalMovDegOffset = SOWAN.normalMovDegOffset;

	acc = SOWAN.acc;
	spd = SOWAN.spd;
	//spd = MaxLineSpd * Weel_spd_meter_sec_2_RPM;
	turnRate = SOWAN.turnRate;
	printf("acc =%f,spd =%f,turnRate =%f\n", acc, spd, turnRate);

	BackMovEscapeDistance = SOWAN.BackMovEscapeDistance;

	
	printf("BackMov =%.2f \n", BackMovEscapeDistance);

	PatrolCycleCount = SOWAN.PatrolCycleCount;
	PatrolCycleWatiTime = SOWAN.PatrolCycleWatiTime;
	PatrolCycleCount_Work = PatrolCycleCount;
	PatrolCycleWatiTime_Work = PatrolCycleWatiTime;
	printf("PatrolCycle Count = %d,WatiTime = %.2f  \n", PatrolCycleCount, PatrolCycleWatiTime);

	PatrolOverDeg = SOWAN.PatrolOverDeg;
	PatrolOverDeg_FinDeg = SOWAN.PatrolOverDeg_FinDeg;
	printf("PatrolOverDeg = %.2f,PatrolOverDeg_FinDeg = %.2f  \n", PatrolOverDeg, PatrolOverDeg_FinDeg);

	GetGobalPathTest = SOWAN.Debug_SW;

	cv::Point SOWANtmpPoint;
	for (int i = 0; i < SOWAN.Point_Route.size(); i += 2) {
		SOWANtmpPoint = cv::Point(SOWAN.Point_Route.at(i), SOWAN.Point_Route.at(i + 1));
		PatrolPoint.push_back(SOWANtmpPoint);
	}
	printf("PatrolPoint size = %d\n", PatrolPoint.size());

	for (int i = 0; i < SOWAN.Point_Charge.size(); i += 2) {
		SOWANtmpPoint = cv::Point(SOWAN.Point_Charge.at(i), SOWAN.Point_Charge.at(i + 1));
		ChargePoint.push_back(SOWANtmpPoint);
	}
	printf("ChargePoint size = %d\n", ChargePoint.size());

	std::vector<cv::Point> vectortmp;
	vectortmp.assign(4, cv::Point(0, 0));
	for (int i = 0; i < SOWAN.Point_Room.size(); i += 8) {
		for (int j = 0; j < 8; j += 2) {
			SOWANtmpPoint = cv::Point(SOWAN.Point_Room.at(i + j), SOWAN.Point_Room.at(i + j + 1));
			vectortmp.at(j / 2) = SOWANtmpPoint;
		}
		RoomSetPoint.push_back(vectortmp);
	}
	printf("RoomSetPoint size = %d\n", RoomSetPoint.size());

	WorkLoopTime = SOWAN.WorkLoopTime;
	WorkLoopSleepTime = SOWAN.WorkLoopSleepTime;
	WorkLoopTime_Work = WorkLoopTime;
	WorkLoopSleepTime_Work = WorkLoopSleepTime;
	ChargeWaitTime = SOWAN.ChargeWaitTime;
	printf("WorkLoopTime = %.2f , SleepTime = %.2f, ChargeTime = %.2f\n", WorkLoopTime , WorkLoopSleepTime , ChargeWaitTime);

	Motor.Debug_ShowReadError = SOWAN.Debug_ShowReadError;
	LowSensorInstall = SOWAN.LowSensorInstall;

	cv::Mat IMG = imread(SOWAN.BlockImgPath);
	Map_Router.BlockMapInit(&IMG);
	printf("AllMapCheck is %d\n", Map_Router.AllMapCheck());

	SetSystemInitStep(SystemInitStep::fu_SowanParaSetting);

	HitObjectRunawayType = SOWAN.HitObjectRunawayType;

	//imshow("SOWAN.BlockImgPath",IMG);

}
void main_function() {

	////
	SelfPosUpdate(true, FPS_Calculate());
	////
	EtherS.HitCount = (char)normalMovHitObjCount+'a';

	ModeControlSystem();

	if (AutoMove) {
		AutoMoveFunction_Test();
	}

	LoopWorkTimeCounter();
	
	switch (ModeNow) {
	case SystemMode::StandbyMode:
		if (StandbyFunction() == true) {
			getModeChangeCmd = true;
		}
		break;
	case SystemMode::StandbyWithTimeMode:
		if (WaitTimeFunction() == true) {
			getModeChangeCmd = true;
			WaitProcessWorking = false;
		}
		break;
	case SystemMode::PatrolMode:
		if (PatrolMoveFunction() == true) {
			getModeChangeCmd = true;
			PatrolProcessWorking = false;
			GoStartPosWorking = true;
			patrolMovTagCount = 0;
			printf("ModeNow  %d WorkEnd ,PC = %d \n", ModeNow, patrolMovTagCount);
		}
		break;
	case SystemMode::GoRoomMode:
		if (InterruptGotoRoom(SelRoomID) == GotoRoomCase::EndRoomProcess) {
			getModeChangeCmd = true;
			RoomProcessWorking = false;
			GoStartPosWorking = true;
			GotoRoomCaseDir = 0;
			printf("ModeNow  %d WorkEnd \n", ModeNow);
		}
		break;
	case SystemMode::GoChargeMode:
		if (InterruptGotoCharge() == GotoChargPosCase::EndChargPosProcess) {
			getModeChangeCmd = true;
			GoChargePosWorking = false;
			GoChargePointDir = 0;
			printf("ModeNow  %d WorkEnd \n", ModeNow);
		}
		break;
	case SystemMode::GoStartPosMode:
		if (GoStartPosFunction() == true) {
			getModeChangeCmd = true;
			GoStartPosWorking = false;
			GoStartPosPointCount = 0;
			printf("ModeNow  %d WorkEnd \n", ModeNow);
		}
	}


	if ( FrontObjectClose && HitObjectStop && (lastMovCmd == 'w' || lastMovCmd == 'q' || lastMovCmd == 'e')) {
		MotorMoveControl('s');
	}
	if (BackObjectClose && HitObjectStop && (lastMovCmd == 'x')) {
		//printf("(BackObject Stop Motor. \n");
		MotorMoveControl('s');
	}

	ALL_Monitor_Combine();
	cv::imshow("ALL_Monitor", ALL_Monitor);
	//cv::imshow("Map_Monitor", Map_Monitor);
	//if (!Map_Router.Monitor.empty()) {
	//	
	//	cv::imshow("Map_Router.Monitor", Map_Router.Monitor);
	//}
	//cv::imshow("Para_Monitor", Para_Monitor);
	//cv::imshow("Laser_Monitor", Laser_Monitor);

	char ttt = cv::waitKey(1);
	if (UserGuiMode != 0 ) {

		GUI_control(ttt);

		GUI_control(EtherS.SwitchSel);
		EtherS.SwitchSel = -1;

		User_GUI_control(switchWindowsSel);
		switchWindowsSel = -1;

		User_GUI_control(EtherS.RoomSel);
		EtherS.RoomSel = -1;
	}
	else {
		GUI_control(ttt);
	}
}

int normal_Mov_Fuction(cv::Point2d Tag, double ArrRange
	, double TurnNeedDeg, double CircleNeedDeg
	, double MoveDis, double DegDiff) 
{
	double fabs_work_deg = fabs(DegDiff);
	double MoveDis_DEG = MoveDis * sin(fabs_work_deg * CV_PI / 180.0);
	int arrived = 0;
	DegDiff_SHOW = DegDiff;

	int normalMoveType_in = 0;

	if (normal_Mov_ShowFrist) {
		normal_Mov_ShowFrist = false;
		//printf("Tag(%.2f,%.2f), Arr %.2f, TNDeg %.2f, CNDeg %.2f, MoveDis %.2f, DegDiff %.2f. \n", 
		//	Tag.x,Tag.y, ArrRange, TurnNeedDeg, CircleNeedDeg, MoveDis, DegDiff);
	}

	if (normalMoveType) {
		normalMoveType_in = 5;
	}
	if (MoveDis <= ArrRange) {
		//Arrival
		//printf("Mov_Fuction Arrival, Tag(%.2f,%.2f), ArrRange %.2f, MoveDis %.2f, DegDiff %.2f. \n" ,
		//	Tag.x, Tag.y, ArrRange, MoveDis, DegDiff);
		arrived = 1;
		normal_Mov_ShowFrist = true;
		MotorMoveType2(0, 0);
	}
	else if (fabs_work_deg > TurnNeedDeg) {
		//Turn Move
		if (DegDiff > 0) {
			MotorMoveType2(0, MaxTurnSpd);
		}
		else {
			MotorMoveType2(0, -MaxTurnSpd);
		}
	}
	else if ( (MoveDis > ArrRange) && 
		((fabs_work_deg > CircleNeedDeg ) 
			|| ( ((normalMoveType_in * fabs_work_deg)  > CircleNeedDeg  && (MoveDis > 2) )))
		){
		//Circle Move

		if ( FrontObjectClose 
			// || (GetCircleCenterDistance(Tag, DegDiff) <= (MaxTurnRadius - ArrRange) )
			) {
			//Hit wall or shoter than MaxTurnRadius,change to Turn Mov
			if (DegDiff > 0) {
				MotorMoveType2(0, MaxTurnSpd);
			}
			else {
				MotorMoveType2(0, -MaxTurnSpd);
			}
		}
		else {
			float CircleRate = 0;
			if (normalMoveType == 1) {
				CircleRate = DegDiff / TurnNeedDeg;
			}
			else if (normalMoveType == 2) {
				CircleRate = (0.9 * (DegDiff / TurnNeedDeg));
				if (DegDiff > 0) {
					CircleRate += 0.1;
				}
				else if (DegDiff < 0) {
					CircleRate -= 0.1;
				}
			}
			else {
				//CircleRate = (DegDiff / TurnNeedDeg) * 0.95;
				CircleRate = ((DegDiff + normalMovDegOffset) / TurnNeedDeg);
				//if (DegDiff > 0) {
				//	CircleRate = ((DegDiff - CircleNeedDeg) / TurnNeedDeg);
				//}
				//else if (DegDiff < 0) {
				//	CircleRate = ((DegDiff + CircleNeedDeg) / TurnNeedDeg);
				//}
			}
			
			CircleRate_SHOW = CircleRate;
			MotorMoveType2(MaxLineSpd, MaxCircleSpd * CircleRate);
		}
	}
	else if (MoveDis > ArrRange && !FrontObjectClose) {
		//Fwd Move
		MotorMoveType2(MaxLineSpd, 0);
	}
	else if (MoveDis > ArrRange && FrontObjectClose) {
		MotorMoveType2(0, 0);
		if (normalMovHitObjCount == 0) {
			printf("HitObjCount Happen. \n");
		}
		normalMovHitObjCount++;
		printf("HitObjCount =  %d \r", normalMovHitObjCount);
	}
	return arrived;

}
int normal_Mov(cv::Point2d Tag) {
	normalMovTag = Tag;
	double DegDiff = GetTagDegDiff(Tag);
	if (normalMovHitObjFuc(HitObjectRunawayType, Tag, DegDiff) > 5) {
		return 0;
	}

	double MoveDis = TranFunc.targetDistanceReturn(Tag.x, Tag.y);
	return normal_Mov_Fuction(Tag, ArrivalRange, MinTurnNeedDeg, MinCircleNeedDeg, MoveDis, DegDiff);
}
int normal_Mov_Fu(cv::Point2d Tag) {
	normalMovTag = Tag;
	double DegDiff = GetTagDegDiff(Tag);
	if (normalMovHitObjFuc(HitObjectRunawayType, Tag, DegDiff) > 5) {
		return 0;
	}

	double MoveDis = TranFunc.targetDistanceReturn(Tag.x, Tag.y);
	return normal_Mov_Fuction(Tag, 0.1, 10, 3, MoveDis, DegDiff);

}

int Base_Mov(cv::Point2d Tag) {

	int target_arr = 0;
	double MoveDis = TranFunc.targetDistanceReturn(Tag.x, Tag.y);

	double Base_Mov_Arrival = ArrivalRange * 0.7;

	if (Base_Mov_Arrival < 0.1) {
		Base_Mov_Arrival = 0.1;
	}

	if ((MoveDis > Base_Mov_Arrival) || BaseMovTagCaseDir) {

		double MoveDir = TranFunc.targetDegReturn(Tag.x, Tag.y);
		double MoveDirTheta = MoveDir * CV_PI / 180.0;
		double re_w = cos(MoveDirTheta / 2.0);
		double re_z = sin(MoveDirTheta / 2.0);

		switch (BaseMovTagCaseDir) {
		case BaseMovCase::SetTag:
			if (re_w > 0 && re_z < 0 && re_w < 0.5) {
				re_w = -re_w;
				re_z = -re_z;
			}
			BaseMovgoal.pose.position.x = Tag.x;
			BaseMovgoal.pose.position.y = Tag.y;
			BaseMovgoal.pose.position.z = 0;
			BaseMovgoal.pose.orientation.x = 0;
			BaseMovgoal.pose.orientation.y = 0;
			BaseMovgoal.pose.orientation.w = re_w;
			BaseMovgoal.pose.orientation.z = re_z;

			goal_pub_switch = true;
			BaseMovTagCaseDir = 1;
			break;
		case BaseMovCase::WaitGoalPub:
			if (goal_pub_switch == false) {
				BaseMovTagCaseDir = 2;
			}
			break;
		case BaseMovCase::GetVelThenMov:
			if (MoveDis > Base_Mov_Arrival) {
				//MotorMoveType2(g_vel, t_vel);
				if (BaseMovEsc == 1) {
					if (Motor.Motor_State_Read(1) == 3 && Motor.Motor_State_Read(2) == 3) {
						BaseMovEsc = 0;
					}
					else {
						break;
					}
				}
				if (FrontObjectClose) {
					if (t_vel) {
						MotorMove(0, t_vel);
					}
					else if (t_vel == 0) {
						MotorMove(0, 0);
						BaseMovEsc = 1;
						MotorDirPlusMove('x', 1000 * BackMovEscapeDistance);
					}
				}
				else {
					MotorMove(g_vel, t_vel);
				}
			}
			else {
				goal_cancel_pub = true;
				BaseMovTagCaseDir = 3;
			}
			break;
		case BaseMovCase::ClearDir:
			BaseMovTagCaseDir = 0;
			target_arr = 1;
			break;
		}
	}
	else {
		MotorMoveType2(0, 0);
		BaseMovTagCaseDir = 0;
		target_arr = 1;
	}

	return target_arr;

}
int normal_Base_Mov_Mix(cv::Point2d Tag) {
	int returnValue = 0;

	switch (normalMovAdvCaseDir) {
	case normalMovAdvCase::MoveByNormal:
		if (normal_Mov(Tag) == 1) {
			normalMovAdvCaseDir = normalMovAdvCase::MoveEnd;
		}
		if (normalMovHitObjCount == 20) {
			normalMovAdvCaseDir = normalMovAdvCase::NormalFailBackMov;
			printf("NormalFail BackMov \n");
			normalMovHitObjCount = 0;
		}
		break;
		//
	case normalMovAdvCase::NormalFailBackMov:

		if (OneMotionMov('x', 1000 * BackMovEscapeDistance) == true) {
			normalMovAdvCaseDir = normalMovAdvCase::MoveByBaseMov;
			printf("BaseMov Start \n");
		}
		break;
	case normalMovAdvCase::MoveByBaseMov:
		if (Base_Mov(Tag) == 1) {
			normalMovAdvCaseDir = normalMovAdvCase::MoveEnd;
		}
		break;
	case normalMovAdvCase::MoveEnd:
		normalMovAdvCaseDir = 0;
		returnValue = 1;
		break;
	}
	return returnValue;
}

bool FaceTag(cv::Point2d Tag) {
	bool arrived = false;

	double DegDiff = GetTagDegDiff(Tag);
	double fabs_work_deg = fabs(DegDiff);


	if (fabs_work_deg > 3) {
		double turnspd = 1;
		if (fabs_work_deg < 10)
			turnspd = 0.5;

		if (DegDiff > 0) {
			MotorMoveType2(0, MaxTurnSpd * turnspd);
		}
		else {
			MotorMoveType2(0, -MaxTurnSpd * turnspd);
		}
	}
	else {
		MotorMoveType2(0, 0);
		arrived = true;
	}

	return arrived;
}
int normalMovHitObjFuc(int type, cv::Point2d Tag, double DegDiff) {

	double fabs_work_deg = fabs(DegDiff);
	int Weel_L_State = Motor.Motor_State_Read(1);
	int Weel_R_State = Motor.Motor_State_Read(2);

	if (normalMovHitObjCount > 13) {
		if (fabs_work_deg <= (MinCircleNeedDeg*0.3)) {
			MotorMoveType2(0, 0);
			if (FrontObjectClose && Weel_R_State == 3 && Weel_L_State == 3) {
				normalMovHitObjCount++;
				if (normalMovHitObjCount > 20) {
					if (FrontObject_LowOnly && !FrontObject_Laser) {
						normalMovHitObjCount = 21;
						if (LowSensorInstall) {
							VehicleStopVoicePlay(SOWAN.VehicleStopVoicePath, SOWAN.VehicleStopVoiceTime);
						}
						

					}
					else {
						normalMovHitObjCount = 20;
					}
				}
				else {
					if (normalMovHitObjCount == 20) {
						printf("HitObjCount= %2d / 20, Tag = %d, Deg=%3.2f_%3.2f, (%.2f,%.2f)\n",
							normalMovHitObjCount,
							patrolMovTagCount % PatrolPoint.size(),
							DegDiff,
							fabs_work_deg,
							Tag.x, Tag.y
						);
						if (LowSensorInstall) {
							VehicleStopVoiceInit(SOWAN.VehicleStopVoicePath, SOWAN.VehicleStopVoiceTime);
						}
						
					}
					else {
						printf("HitObjCount= %2d / 20 , Tag = %d, Deg=%3.2f_%3.2f, (%.2f,%.2f)\r",
							normalMovHitObjCount,
							patrolMovTagCount % PatrolPoint.size(),
							DegDiff,
							fabs_work_deg,
							Tag.x, Tag.y
						);
					}
				}
			}
			else if (!FrontObjectClose && Weel_R_State == 3 && Weel_L_State == 3) {
				normalMovHitObjCount = 0;
			}
		}
		else {
			//Turn Move
			printf("HitObjCount %2d, DegDiff %3.2f \r", normalMovHitObjCount, DegDiff);
			if (DegDiff > 0) {
				MotorMoveType2(0, MaxTurnSpd * 0.5);
			}
			else {
				MotorMoveType2(0, -MaxTurnSpd * 0.5);
			}
		}
	}
	else if (normalMovHitObjCount > 9) {
		//wait 360 turn move end
		if (Weel_R_State == 3 && Weel_L_State == 3) {
			if (!FrontObjectClose) {
				normalMovHitObjCount = 0;
				MotorMoveType2(0, 0);
				printf("HitObj Out ,normalMovHitObjCount reach %d \n", normalMovHitObjCount);
			}
			else if (FrontObjectClose) {
				normalMovHitObjCount++;
				printf("HitObjCount reach %d \n", normalMovHitObjCount);
			}
		}
	}
	else if (normalMovHitObjCount == 9) {
		if (OneMotionMov(normalMovHitObj_Dir, normalMovHitObj_Value, normalMovHitObj_Spd, acc)) {
			normalMovHitObjCount = 10;
			printf("HitObjCount reach %d \n", normalMovHitObjCount);
		}
	}
	else if (normalMovHitObjCount == 8) {
		if (OneMotionMov(normalMovHitObj_Dir, normalMovHitObj_Value, normalMovHitObj_Spd, acc)) {
			normalMovHitObj_Spd = spd * turnRate;
			normalMovHitObj_Dir = 'a';
			normalMovHitObj_Value = 360;
			normalMovHitObjCount = 9;
			printf("Spd %d, Dir %c, Act %d ,Value %d\n", normalMovHitObj_Spd, normalMovHitObj_Dir, normalMovHitObjCount, normalMovHitObj_Value);
		}
	}
	else if (normalMovHitObjCount == 7) {
		if (OneMotionMov(normalMovHitObj_Dir, normalMovHitObj_Value, normalMovHitObj_Spd,acc)) {
			normalMovHitObj_Spd = spd;
			normalMovHitObj_Dir = 'x';			
			normalMovHitObjCount = 8;
			int BackValue = ((FrontSafeDistance - FrontObjectClose_Value_Save) * 1000);
			if (BackValue > 30) {
				normalMovHitObj_Value = BackValue - 30;
			}
			else if (BackValue <= 30 && BackValue > 0) {
				normalMovHitObj_Value = 1;
			}
			else {
				normalMovHitObj_Value = 100;
			}
			printf("Spd=%d, Dir=%c, Act=%d, Val=%d, FoObjVal=%.1f, FoObjVal_S=%.1f \n"
				, normalMovHitObj_Spd, normalMovHitObj_Dir, normalMovHitObjCount
				, normalMovHitObj_Value, FrontObjectClose_Value, FrontObjectClose_Value_Save);
		}
	}
	else if (normalMovHitObjCount == 6) {
		if (type == 1) {
			printf("HitObjCount reach %d ,Do Back ,Motor(%d,%d)\n", normalMovHitObjCount, Weel_L_State, Weel_R_State);
			FrontObjectClose_Value_Save = FrontObjectClose_Value;
			if (FrontObjectClose_L && !FrontObjectClose_R) {
				normalMovHitObj_Spd = spd * turnRate;
				normalMovHitObj_Dir = 'a';
				normalMovHitObj_Value = 10;
				normalMovHitObjCount = 7;
			}
			else if (!FrontObjectClose_L && FrontObjectClose_R) {
				normalMovHitObj_Spd = spd * turnRate;
				normalMovHitObj_Dir = 'd';
				normalMovHitObj_Value = 10;
				normalMovHitObjCount = 7;
			}
			else {
				normalMovHitObj_Spd = spd;
				normalMovHitObj_Dir = 'x';				
				normalMovHitObjCount = 8;
				int BackValue = ((FrontSafeDistance - FrontObjectClose_Value_Save) * 1000);
				if (BackValue > 30) {
					normalMovHitObj_Value = BackValue - 30;
				}
				else if (BackValue <= 30 && BackValue > 0) {
					normalMovHitObj_Value = 1;
				}
				else {
					normalMovHitObj_Value = 100;
				}
			}
			printf("Spd=%d, Dir=%c, Act=%d, Val=%d, FoObjVal=%.1f, FoObjVal_S=%.1f \n"
				, normalMovHitObj_Spd, normalMovHitObj_Dir, normalMovHitObjCount
				, normalMovHitObj_Value, FrontObjectClose_Value, FrontObjectClose_Value_Save);

		}
		else {
			//Do 360 Turn Mov
			printf("HitObjCount reach %d ,Do 360 turn,Motor(%d,%d)\n", normalMovHitObjCount, Weel_L_State, Weel_R_State);
			normalMovHitObj_Spd = spd * turnRate;
			normalMovHitObj_Dir = 'a';
			normalMovHitObj_Value = 360;
			normalMovHitObjCount = 9;
			printf("Spd=%d, Dir=%c, Act=%d, Val=%d, FoObjVal=%.1f, FoObjVal_S=%.1f \n"
				, normalMovHitObj_Spd, normalMovHitObj_Dir, normalMovHitObjCount
				, normalMovHitObj_Value, FrontObjectClose_Value, FrontObjectClose_Value_Save);
		}
	}
	else if (!FrontObjectClose && normalMovHitObjCount <= 5) {
		normalMovHitObjCount = 0;
		StopOtherVoice = false;
	}

	return normalMovHitObjCount;

}

void AutoMoveFunction_Test() {
	if (baseControlMode == 1) {
		if (PatrolPoint.size() > 0) {
			cv::Point2d real = GetPatrolPoint(patrolMovTagCount);
			if (normal_Mov(real)) {
				patrolMovTagCount++;
			}
		}
	}
	else if (baseControlMode == 2) {
		normal_Mov(normalMovTag);
	}
	else if (baseControlMode == 3) {
		if (PatrolPoint.size() > 0) {
			cv::Point2d real = GetPatrolPoint(patrolMovTagCount);
			if (Base_Mov(real) == 1) {
				patrolMovTagCount++;
			}
		}
	}
	else if (baseControlMode == 4) {
		Base_Mov(normalMovTag);
	}
	else if (baseControlMode == 5) {
		if (PatrolPoint.size() > 0) {
			cv::Point2d real = GetPatrolPoint(patrolMovTagCount);
			if (normal_Base_Mov_Mix(real) == 1) {
				patrolMovTagCount++;
			}
		}
	}
	else if (baseControlMode == 6) {
		normal_Base_Mov_Mix(normalMovTag);
	}
}
bool PatrolMoveFunction() {
	bool returnvalue = false;
	double DegDiff;

	if (PatrolPoint.size() > 0) {
		if (SOWAN.PatrolVoiceSwitch) {
			SystemVoicePlay(SOWAN.PatrolVoicePath, SOWAN.PatrolVoiceTime);
		}
		cv::Point2d real = GetPatrolPoint(patrolMovTagCount);

		if (!PatrolMov_NextOverDeg_SW) {
			if (normal_Mov(real) == 1) {
				patrolMovTagCount++;
				if (patrolMovTagCount >= PatrolPoint.size()) {
					MotorMoveType2(0, 0);
					returnvalue = true;
					printf("PatrolMoveFunction (%d / %d) End, returnvalue %d. \n", patrolMovTagCount, PatrolPoint.size(), returnvalue);
				}
				else {
					printf("PatrolMovePoint (%d / %d) End. \n", patrolMovTagCount, PatrolPoint.size());
					real = GetPatrolPoint(patrolMovTagCount);
					DegDiff = fabs(GetTagDegDiff(real));
					if (DegDiff >= PatrolOverDeg) {
						printf("PatrolMovePoint ,DegDiff(%.2f) >= PatrolOverDeg(%.2f). \n", DegDiff, PatrolOverDeg);
						PatrolMov_NextOverDeg_SW = true;
						MotorMoveType2(0, 0);
					}
				}
			}
		}
		else {
			double tmpf = GetTagDegDiff(real);
			double MoveDis = TranFunc.targetDistanceReturn(real.x, real.y);
			DegDiff = fabs(tmpf);
			printf("Patrol Face Self(%.2f,%.2f)toTag(%.2f,%.2f), DegDiff %.2f, MoveDis %.2f \n", 
				TranFunc.TF_Pos.Pos.x, TranFunc.TF_Pos.Pos.y, real.x, real.y, tmpf, MoveDis);

			if ( (FaceTag(real) == true) || (DegDiff < PatrolOverDeg_FinDeg) || (MoveDis <= ArrivalRange) ) {
				PatrolMov_NextOverDeg_SW = false;
				printf("Patrol Self(%.2f,%.2f)toFaceTag(%.2f,%.2f), DegDiff %.2f ,MoveDis %.2f End.\n", 
					TranFunc.TF_Pos.Pos.x, TranFunc.TF_Pos.Pos.y,real.x, real.y, tmpf, MoveDis);
			}
		}

	}
	else {
		returnvalue = true;
		printf("PatrolMoveFunction  %d nothing out %d\n", patrolMovTagCount, returnvalue);
	}
	
	return returnvalue;


}
bool GoStartPosFunction() {
	bool return_value = false;
	if (GetGoStartPos == false) {
		GetGoStartPos = SetGoStartPos();
		return_value = false;
	}
	else if (GoStartPosPoint.size() != 0) {
		cv::Point2d real = GoStartPosPoint[GoStartPosPointCount];
		if (normal_Mov_Fu(real) == 1) {
			GoStartPosPointCount++;
			return_value = false;
		}
		if (GoStartPosPointCount == GoStartPosPoint.size()) {
			MotorMoveType2(0, 0);
			GoStartPosPointCount = 0;
			return_value = true;
		}
	}
	else{
		MotorMoveType2(0, 0);
		GoStartPosPointCount = 0;
		return_value = true;
	}
	return return_value ;
}
int InterruptGotoRoom(int RoomID) {
	cv::Point2d TagPos;
	cv::Point2d real;

	//Press button halfway
	if ((UserStopRoomProcess || (GPIO_Control.GPIO_Read_Float(0) >= 1) ) 
		&& ( (normalMovHitObjCount == 20) || (DoorOpenCounter == 10) )) {
		bool Work_tmp = false;
		switch (GotoRoomCaseDir) {
		case GotoRoomCase::GoToDoorFrontPos:
			GotoRoomCaseDir = GotoRoomCase::WaitRoomCaseEnd;
			GoFrontDoorPointCount = 0;
			Work_tmp = true;
			break;
		case GotoRoomCase::WatiDoorOpen:
		case GotoRoomCase::GoToInsideDoorPos:
			GotoRoomCaseDir = GotoRoomCase::GoToDoorFrontPos_II;
			Work_tmp = true;
			break;
		case GotoRoomCase::GoToStandbyPos:
			GotoRoomCaseDir = GotoRoomCase::GoToInsideDoorPos_II;
			Work_tmp = true;
			break;
		}

		if (Work_tmp) {
			MotorMoveType2(0, 0);
			UserStopRoomProcess = false;
			printf("UserStopRoomProcess %d,GPIO %.0f,normalMovHitObjCount %d,DoorOpenCounter %d \n"
				, UserStopRoomProcess, GPIO_Control.GPIO_Read_Float(0) , normalMovHitObjCount, DoorOpenCounter);
			printf("GotoRoomCase(%d)_%s, Interrupt End\n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			VoicePlay(Home_Root_String + "/Voice/DefaultVoice.wav");
			usleep(int(1000000 * 5));
		}
	}

	//select Point
	switch (GotoRoomCaseDir) {
	case GotoRoomCase::GoToDoorFrontPos:
	case GotoRoomCase::GoToDoorFrontPos_II:
		TagPos = GetRoomPoint(RoomID, 0);
		break;
	case GotoRoomCase::GoToInsideDoorPos:
	case GotoRoomCase::GoToInsideDoorPos_II:
	case GotoRoomCase::FaceToDoor:
	case GotoRoomCase::FaceInsideDoor:
		TagPos = GetRoomPoint(RoomID, 1);
		break;
	case GotoRoomCase::GoToStandbyPos:
		TagPos = GetRoomPoint(RoomID, 2);
		break;
	case GotoRoomCase::FaceToBed:
		TagPos = GetRoomPoint(RoomID, 3);
		break;
	}

	//Main Function
	switch (GotoRoomCaseDir) {
	case GotoRoomCase::WaitRobotReady:
		MotorMoveType2(0, 0);

		if ((Motor.Motor_State_Read(1) == 3) && (Motor.Motor_State_Read(2) == 3)) {
			if (SetGoFrontPos(RoomID) == true) {
				GotoRoomCaseDir = GotoRoomCase::GoToDoorFrontPos;
				printf("GotoRoomCase(%d)_%s \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);

				//ProgramVoiceTime = ros::Time::now();
				SystemVoiceInit(1, SOWAN.RoomVoice.at(RoomID), SOWAN.RoomVoiceTime.at(RoomID));

				GoFrontDoorPointCount = 0;
				printf("GoFrontDoorPointCount = %d / %d\n", GoFrontDoorPointCount, GoFrontDoorPoint.size());
			}
		}
		break;
	case GotoRoomCase::GoToDoorFrontPos:
		SystemVoicePlay(SOWAN.RoomVoice.at(RoomID), SOWAN.RoomVoiceTime.at(RoomID));

		if (GoRoomByNormalMov) {
			if (GoFrontDoorPointCount < GoFrontDoorPoint.size()) {
				cv::Point2d real = GoFrontDoorPoint[GoFrontDoorPointCount];
				if (normal_Mov_Fu(real) == 1) {
					GoFrontDoorPointCount++;
					printf("GoFrontDoorPointCount = %d / %d\n", GoFrontDoorPointCount, GoFrontDoorPoint.size());
				}
			}
			if (GoFrontDoorPointCount == GoFrontDoorPoint.size()) {
				MotorMoveType2(0, 0);
				GotoRoomCaseDir++;
				GoFrontDoorPointCount = 0;
				printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			}
		}
		else {
			if (Base_Mov(TagPos) == 1) {
				MotorMoveType2(0, 0);
				GotoRoomCaseDir++;
				printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			}
		}

		break;
	case GotoRoomCase::GoToInsideDoorPos:
	case GotoRoomCase::GoToStandbyPos:
	case GotoRoomCase::GoToInsideDoorPos_II:
	case GotoRoomCase::GoToDoorFrontPos_II:

		if (GoRoomByNormalMov) {
			if (normal_Mov_Fu(TagPos) == 1) {
				MotorMoveType2(0, 0);
				GotoRoomCaseDir++;
				printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			}
		}
		else {
			if (Base_Mov(TagPos) == 1) {
				MotorMoveType2(0, 0);
				GotoRoomCaseDir++;
				printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			}
		}
		break;
	case GotoRoomCase::FaceToDoor:
	case GotoRoomCase::FaceToBed:
	case GotoRoomCase::FaceInsideDoor:
		if (FaceTag(TagPos) == true) {
			MotorMoveType2(0, 0);
			GotoRoomCaseDir++;
			printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
		}
		break;
	case GotoRoomCase::SentDoorOpenSignal:
		SetRemoteAdress(0x80);
		SetRemoteSwitch(true);

		if (RoomID == 8) {
			GPIO_Control.GPIO_Set_Value(10, 1);
		}
		usleep( int(1000000 * SOWAN.DoorOpenWaitTime));
		
		SetRemoteSwitch(false);
		SetRemoteAdress(0);
		if (RoomID == 8) {
			GPIO_Control.GPIO_Set_Value(10,0);
		}
		SetCameraRec(true);
		GotoRoomCaseDir++;
		printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);

		break;
	case GotoRoomCase::WatiDoorOpen:
		if (!DoorClosed) {
			DoorOpenCounter = 0;
			GotoRoomCaseDir = GotoRoomCase::GoToInsideDoorPos;
			printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
		}
		else {
			if (DoorOpenCounter < 10) {
				DoorOpenCounter++;
			}
		}
		break;
	case GotoRoomCase::WaitUser:
		if (UserStopRoomProcess || (GPIO_Control.GPIO_Read_Float(0) >= 1)) {
			VoicePlay(Home_Root_String + "/Voice/DefaultVoice.wav");
			usleep(int(1000000 * 5));
			GotoRoomCaseDir++;
			printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
			UserStopRoomProcess = false;
		}
		break;
	case GotoRoomCase::WaitRoomCaseEnd:
		MotorMoveType2(0, 0);
		SetCameraRec(false);
		GotoRoomCaseDir = GotoRoomCase::EndRoomProcess;
		printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
		UserStopRoomProcess = false;
		DoorOpenCounter = 0;
		break;
	case GotoRoomCase::EndRoomProcess:
		MotorMoveType2(0, 0);
		GotoRoomCaseDir = GotoRoomCase::EndRoomProcess;
		printf("GotoRoomCase(%d)_%s  \n", GotoRoomCaseDir, GotoRoomCaseString[GotoRoomCaseDir]);
		break;
	}
	return GotoRoomCaseDir;

}
int InterruptGotoCharge() {
	cv::Point2d TagPos;

	switch (GoChargePointDir) {
	case GotoChargPosCase::Charge_WaitRobotReady:
		MotorMoveType2(0, 0);
		if ((Motor.Motor_State_Read(1) == 3) && (Motor.Motor_State_Read(2) == 3)) {
			if (SetGoChargePos() == true) {
				GoChargePointDir = GotoChargPosCase::GoToChargPos;
				GoChargePointCount = 0;
				printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
			}
		}
		break;
	case GotoChargPosCase::GoToChargPos:
		if (GoChargePointCount < Go_ChargePoint.size()) {
			cv::Point2d real = Go_ChargePoint[GoChargePointCount];
			if (normal_Mov_Fu(real) == 1) {
				GoChargePointCount++;
				printf("GoChargePointCount = %d / %d\n", GoChargePointCount, Go_ChargePoint.size());
			}
		}
		if (GoChargePointCount == Go_ChargePoint.size()) {
			MotorMoveType2(0, 0);
			GoChargePointDir++;
			printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
			GoChargePointCount = 0;
		}
		break;
	case GotoChargPosCase::FaceToChargPos:
		TagPos = TranFunc.Img2Real(ChargePoint[1] - cv::Point(Map_Rect.x, Map_Rect.y));
		if (FaceTag(TagPos) == true) {
			MotorMoveType2(0, 0);
			GoChargePointDir = GotoChargPosCase::CallCamToWork;
			printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
		}
		break;
	case GotoChargPosCase::CallCamToWork:
		cam_Trac_Working = true;
		CallCamWork = 1;
		GoChargePointDir = GotoChargPosCase::WaitCamWorkEnd;
		printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
		break;
	case GotoChargPosCase::WaitCamWorkEnd:
		GoCharegPointProcess();
		if (CallCamWork == 0 && CamSayWorking == 0) {
			cam_Trac_Working = false;
			GoChargePointDir = GotoChargPosCase::WaitChargeTimeEnd;
			printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
			WaitChargeTime_Pre = ros::Time::now();
			printf("WaitChargeTime Start %.2f ,NowTime = %.2f ,ChargeWaitTime = %.2f) \n"
				, (ros::Time::now() - WaitChargeTime_Pre).toSec() , ros::Time::now().toSec() , ChargeWaitTime);
		}
		break;
	case GotoChargPosCase::WaitChargeTimeEnd:
		if (((ros::Time::now() - WaitChargeTime_Pre).toSec() > ChargeWaitTime) || RoomProcessWorking) {
			printf("Wait Time  %.2f / %.2f ==> %4.2f %% ,WaitChargeTime End \n",
				(ros::Time::now() - WaitChargeTime_Pre).toSec(),
				ChargeWaitTime,
				100 * ((ros::Time::now() - WaitChargeTime_Pre).toSec() / (ChargeWaitTime + 0.1)));
			GoChargePointDir = GotoChargPosCase::GetDownChargePlatform;
			printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
			//printf("before CharegProcessSpd = %d,CharegProcessAcc = %d\n", CharegProcessSpd, CharegProcessAcc);
			CharegProcessSpd = 150;
			CharegProcessAcc = 200;
			printf("after CharegProcessSpd = %d,CharegProcessAcc = %d\n", CharegProcessSpd, CharegProcessAcc);
		}
		else {
			printf("Wait Time  %.2f / %.2f ==> %.2f %% \r",
				(ros::Time::now() - WaitChargeTime_Pre).toSec(),
				ChargeWaitTime ,
				100 * ((ros::Time::now() - WaitChargeTime_Pre).toSec() / ChargeWaitTime));
		}
		break;
	case GotoChargPosCase::GetDownChargePlatform:
		if (OneMotionMov('x', 750, CharegProcessSpd, CharegProcessAcc) == true) {
			GoChargePointDir = GotoChargPosCase::EndChargPosProcess;
			printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
		}
		break;
	case GotoChargPosCase::EndChargPosProcess:
		ChargeEndGoStart = true;
		GoChargePointDir = GotoChargPosCase::EndChargPosProcess;
		printf("GotoChargPosCase(%d)_%s \n", GoChargePointDir, GotoChargPosString[GoChargePointDir]);
		break;
	}
	return GoChargePointDir;
}
bool StandbyFunction() {
	if (RoomProcessWorking) {
		return true;
	}
	else if (GoChargePosWorking ) {
		if (!GoStartPosWorking && !PatrolProcessWorking && !RoomProcessWorking) {
			return true;
		}
	}
	if (PatrolCycleCount != 0) {
		if (PatrolCycleWatiTime_Work == 0) {
			GoStartPosWorking = true;
			PatrolProcessWorking = true;
			if (PatrolCycleCount_Work > 0) {
				PatrolCycleCount_Work--;
			}
			printf("Standby End, Count = %d \n ", PatrolCycleCount_Work);
			return true;
		}
		else if (PatrolCycleWatiTime_Work > 0 && PatrolCycleCount_Work !=0 ){
			printf("Standby End,GoWaitTime = %.2f, Count = %d \n ", PatrolCycleWatiTime_Work ,  PatrolCycleCount_Work);
			WaitProcessWorking = true;
			WaitModetime_Pre = ros::Time::now();
			return true;
		}
	}
	return false;
}
bool WaitTimeFunction() {
	if (PatrolCycleWatiTime_Work > 0) {
		int timediff = (int)(ros::Time::now() - WaitModetime_Pre).toSec();
		if (timediff > 0) {
			PatrolCycleWatiTime_Work = PatrolCycleWatiTime_Work - timediff;
			WaitModetime_Pre = ros::Time::now();
			printf("WaitTimeFunction %.2f \n ", PatrolCycleWatiTime_Work);
		}
	}
	if (PatrolCycleWatiTime_Work <= 0) {
		
		if (PatrolCycleCount_Work != 0) {
			if (PatrolCycleCount_Work > 0) {
				PatrolCycleCount_Work--;
			}
			GoStartPosWorking = true;
			PatrolProcessWorking = true;
			
			printf("WaitTime Fnc End %.2f, Count = %d \n ", PatrolCycleWatiTime_Work, PatrolCycleCount_Work);
			PatrolCycleWatiTime_Work = PatrolCycleWatiTime;
			return true;
		}
		else {
			printf("WaitTime End %.2f, Count = %d ==== WorkEnd\n ", PatrolCycleWatiTime_Work, PatrolCycleCount_Work);
			return false;
		}
		
	}
	return false;
}
void LoopWorkTimeCounter() {
	if (WorkLoopTime_Work != 0) {
		GetWorkLoopTime = (ros::Time::now() - WorkLoopTime_Ros).toSec();
		//printf("GetWorkLoopTime = %d", GetWorkLoopTime);
		if (NowWorkLoop && WorkLoopTime_Work > 0 && GetWorkLoopTime > WorkLoopTime_Work) {
			WorkLoopTime_Ros = ros::Time::now();
			WorkLoopSleepProcess = true;
			printf("WorkLoopSleepProcess = true ,%.2f \n",GetWorkLoopTime);
			getModeChangeCmd = true;
		}
		else if (!NowWorkLoop && WorkLoopSleepTime_Work > 0 && GetWorkLoopTime > WorkLoopSleepTime_Work) {
			WorkLoopTime_Ros = ros::Time::now();
			WorkLoopSleepProcess = false;
			printf("WorkLoopSleepProcess = false ,%.2f \n", GetWorkLoopTime);
			getModeChangeCmd = true;
		}
	}

}
void ModeControlSystem() {

	//bool RoomProcessWorking = false;
	//bool PatrolProcessWorking = false;
	//bool GoStartPosWorking = false;
	//bool GoChargePosWorking = false;

	if (getModeChangeCmd) {
		/////
		ModeChangeCmd = ModeNow;
		switch (ModeNow) {
		case SystemMode::StandbyMode:
			if (RoomProcessWorking) {
				ModeChangeCmd = SystemMode::GoRoomMode;
			}
			else if (GoChargePosWorking) {
				ModeChangeCmd = SystemMode::GoChargeMode;
			}
			else if (GoStartPosWorking || PatrolProcessWorking) {
				ModeChangeCmd = SystemMode::GoStartPosMode;
			}
			else if (WaitProcessWorking) {
				ModeChangeCmd = SystemMode::StandbyWithTimeMode;
			}
			else if (WorkLoopSleepProcess) {
				ModeChangeCmd = SystemMode::LoopWorkSleepMode;
			}
			break;
		case SystemMode::StandbyWithTimeMode:
			if (RoomProcessWorking) {
				ModeChangeCmd = SystemMode::GoRoomMode;
			}
			else if (GoChargePosWorking) {
				ModeChangeCmd = SystemMode::GoChargeMode;
			}
			else if (GoStartPosWorking || PatrolProcessWorking) {
				ModeChangeCmd = SystemMode::GoStartPosMode;
			}
			break;
		case SystemMode::GoRoomMode:
			if (RoomProcessWorking == false) {
				if (SelRoomIDNext != -1) {
					SelRoomID = SelRoomIDNext;
					SelRoomIDNext = -1;
					RoomProcessWorking = true;
					ModeChangeCmd = SystemMode::StandbyMode;
					GotoRoomCaseDir = 0;
				}
				else {
					ModeChangeCmd = SystemMode::GoStartPosMode;
					GoStartPosWorking = true;
					GotoRoomCaseDir = 0;
				}
			}

			break;
		case SystemMode::GoStartPosMode:
			if (RoomProcessWorking) {
				ModeChangeCmd = SystemMode::GoRoomMode;
			}
			else if (ChargeEndGoStart) {
				ChargeEndGoStart = false;
				ModeChangeCmd = SystemMode::StandbyMode;
			}
			else if (PatrolProcessWorking) {
				ModeChangeCmd = SystemMode::PatrolMode;
			}
			else {
				ModeChangeCmd = SystemMode::StandbyMode;
			}
			GoStartPosPointCount = 0;
			break;
		case SystemMode::GoChargeMode:
			if (GoChargePosWorking == false) {
				if (RoomProcessWorking) {
					ModeChangeCmd = SystemMode::GoRoomMode;
				}
				else {
					ModeChangeCmd = SystemMode::GoStartPosMode;
				}
				GoChargePointDir = 0;
			}
			break;
		case SystemMode::PatrolMode:
			if (RoomProcessWorking) {
				ModeChangeCmd = SystemMode::GoRoomMode;
				patrolMovTagCount = 0;
			}
			else if (PatrolProcessWorking == false){
				ModeChangeCmd = SystemMode::GoStartPosMode;
				patrolMovTagCount = 0;
				if (SOWAN.ChargePlatform) {
					GoChargePosWorking = true;
				}
			}
			break;
		case SystemMode::LoopWorkSleepMode:
			ModeChangeCmd = SystemMode::StandbyMode;
		}
		/////
		printf("Flag_MC=%d,S=%d,P=%d,C=%d\n", getModeChangeCmd, GoStartPosWorking, PatrolProcessWorking, GoChargePosWorking);
		printf("ModeChangeCmd %s(%d) to %s(%d) \n", SystemModeString[ModeNow],ModeNow, SystemModeString[ModeChangeCmd], ModeChangeCmd);
		if (ModeNow != ModeChangeCmd) {
			switch (ModeChangeCmd) {
			case SystemMode::StandbyMode:
			case SystemMode::StandbyWithTimeMode:
				ModeNow = ModeChangeCmd;
				break;
			case SystemMode::PatrolMode:
				SystemVoiceInit(0, SOWAN.PatrolVoicePath, SOWAN.PatrolVoiceTime);
				ModeNow = SystemMode::PatrolMode;
				break;
			case SystemMode::GoRoomMode:
				ModeNow = SystemMode::GoRoomMode;
				GotoRoomCaseDir = 0;
				GetGobalPathDir = 0;
				break;
			case SystemMode::GoChargeMode:
				//SetGoChargePos();
				GetGobalPathDir = 0;
				GoChargePointDir = 0;
				ModeNow = SystemMode::GoChargeMode;
				break;
			case SystemMode::GoStartPosMode:
				GetGobalPathDir = 0;
				GoStartPosPointCount = 0;
				GetGoStartPos = false;
				ModeNow = SystemMode::GoStartPosMode;
				break;
			case SystemMode::LoopWorkSleepMode:
				ModeNow = SystemMode::LoopWorkSleepMode;
				break;
			}
		}
		getModeChangeCmd = false;
	}



}

////Windows click detect function
void mapOnMouse(int Event, int x, int y, int flags, void* param) {
	//if (Event == CV_EVENT_LBUTTONDOWN) {
	//	cv::Point2d real = TranFunc.Img2Real(cv::Point(x, y));
	//	normalMovTag = real;
	//	printf("Mouse_(%d ,%d ) to (%.2f,%.2f)\n", x, y, normalMovTag.x, normalMovTag.y);
	//}
	//if (Event == CV_EVENT_LBUTTONUP) {

	//}

	if (Event == CV_EVENT_LBUTTONDOWN) {
		Start_P = cv::Point(x, y);
		printf("Start_P L_BUTTONDOWN(%d ,%d )\n", x, y);
	}
	if (Event == CV_EVENT_RBUTTONDOWN) {
		End_P = cv::Point(x, y);
		printf("End_P R_BUTTONDOWN(%d ,%d )\n", x, y);
	}
}
void switchOnMouse(int Event, int x, int y, int flags, void* param) {
	if (UserGuiMode != 0 ) {
		//ButtonSizeW ButtonSizeH
		if (Event == CV_EVENT_LBUTTONDOWN) {
			if (x < 400 && y < 150) {
				int dir = (x / 100) + ((y / 50) * 4);
				switchWindowsSel = dir;
				printf("Mouse_(%d ,%d )_button = %d\n", x, y, switchWindowsSel);
			}
			else if (x >= 400 && x < 450 && (y < (RoomSetPoint.size() * 50))) {
				int dir = (y / 50) + 100;
				switchWindowsSel = dir;
				printf("Mouse_(%d ,%d )_button = %d -> RoomSel %d  \n", x, y, switchWindowsSel, dir - 100);
			}
			else {
				switchWindowsSel = -1;
			}
		}
		if (Event == CV_EVENT_LBUTTONUP) {

		}
	}
}

void GUI_control(char ttt) {
	switch (ttt) {
	case 'w':
	case 'x':
	case 'a':
	case 'd':
	case 's':
	case 'q':
	case 'e':
		MotorMoveControl(ttt);
		break;

	case '1':
		Motor.ServoOn(1);
		Motor.ServoOn(2);
		break;
	case '2':
		Motor.ServoOFF(1);
		Motor.ServoOFF(2);
		break;
	case '3':
		spd += 50;
		break;
	case '4':
		spd -= 50;
		break;
	case '5':
		acc += 100;
		break;
	case '6':
		acc -= 100;
		break;
	case '7':
		ManualCouveRate++;
		if (ManualCouveRate > 9)
			ManualCouveRate = 9;
		printf("Set ManualCouveRate = %.4f", ManualCouveRate * 0.075);
		break;
	case '8':
		ManualCouveRate--;
		if (ManualCouveRate < 1 )
			ManualCouveRate = 1;
		printf("Set ManualCouveRate = %.4f", ManualCouveRate * 0.075);
		break;
	case ';':
		UserGuiMode = (UserGuiMode + 1) % UserGuiMode_type;
		break;
	case '[':
		cam_Trac_Working = !cam_Trac_Working;
		break;
	case ']':
		CallCamWork = 1;
		break;
	case 't':
		GPIO_Control.GPIO_inv_Value(0);
		break;
	case 'y':
		GPIO_Control.GPIO_inv_Value(1);
		break;
	case 'u':
		GPIO_Control.GPIO_inv_Value(2);
		break;
	case 'i':
		GPIO_Control.GPIO_inv_Value(3);
		break;
	case 'o':
		GPIO_Control.GPIO_inv_Value(4);
		break;
	case 'p':
		GPIO_Control.GPIO_inv_Value(5);
		break;
	case 'g':
		GPIO_Control.GPIO_inv_Value(6);
		break;
	case 'h':
		GPIO_Control.GPIO_inv_Value(7);
		break;
	case 'j':
		GPIO_Control.GPIO_inv_Value(8);
		break;
	case 'k':
		GPIO_Control.GPIO_inv_Value(9);
		break;
	case 'l':
		GPIO_Control.GPIO_inv_Value(10);
		break;
	case 'm':
		VoicePlay(SOWAN.RoomVoice[0]);
		break;
	case 'n':
		VoicePlay(SOWAN.RoomVoice[1]);
		break;
	case 'b':
		VoicePlay(SOWAN.RoomVoice[2]);
		break;
	}
}
void User_GUI_control(int ttt) {

	if (ttt > 99 && ttt < 209) {

		int SelRoomIDTmp = ttt - 100;
		if (ModeNow != SystemMode::GoRoomMode) {
			SelRoomID = SelRoomIDTmp;
		}
		else if ( SelRoomID != SelRoomIDTmp ){
			SelRoomIDNext = SelRoomIDTmp;
			printf("Not Work Sel %d, Now Work %d ", SelRoomIDNext, SelRoomID);
		}
		ttt = 8;
	}

	switch (ttt) {
	case 0:
		Motor.ServoOn(1);
		Motor.ServoOn(2);
		break;
	case 1:
		Motor.ServoOFF(1);
		Motor.ServoOFF(2);
		break;
	case 2:
		spd += 50;
		break;
	case 3:
		spd -= 50;
		break;
	case 4:
		getModeChangeCmd = true;
		GoStartPosWorking = true;
		PatrolProcessWorking = true;

		break;
	case 5:
		PatrolProcessWorking = false;
		break;
	case 6:
		GoChargePosWorking = true;
		break;
	case 7:
		//acc += 100;
		break;
	case 8:
		//AutoMove = false;
		//patrolMovTagCount = 0;
		//MotorMoveControlType2('s', 0, 0);
		//RoomProcessWorking = true;
		//GotoRoomCaseDir = 0;

		getModeChangeCmd = true;
		RoomProcessWorking = true;

		break;
	case 9:
		if (ModeNow == SystemMode::GoRoomMode) {
		UserStopRoomProcess = true;
		}
		break;
	case 10:
		SetRemoteAdress(1);
		SetRemoteSwitch(true);
		break;
	case 11:
		SetRemoteAdress(1);
		SetRemoteSwitch(false);
		SetRemoteAdress(0);
		break;

	default:
		break;
	}

}

void MotorDirPlusMove(char dir, double Value) {
	double tmp_Plus;
	int Motor_ID_1_Pos = Motor.Motor_Pos_Read(1);
	int Motor_ID_2_Pos = Motor.Motor_Pos_Read(2);

	switch (dir) {
	case 'w':
		tmp_Plus = Value * Weel_plus_per_mm;
		Motor.Motor_Step_Move_Both(1, (int)tmp_Plus, 2, (int)-tmp_Plus);
		OneMotionMovDir_Tag_1 = Motor_ID_1_Pos + tmp_Plus;
		OneMotionMovDir_Tag_2 = Motor_ID_2_Pos - tmp_Plus;
		break;
	case 'x':
		tmp_Plus = Value * Weel_plus_per_mm;
		Motor.Motor_Step_Move_Both(1, (int)-tmp_Plus, 2, (int)tmp_Plus);
		OneMotionMovDir_Tag_1 = Motor_ID_1_Pos - tmp_Plus;
		OneMotionMovDir_Tag_2 = Motor_ID_2_Pos + tmp_Plus;
		break;
	case 'a':
		tmp_Plus = Value * Weel_plus_per_deg;
		Motor.Motor_Step_Move_Both(1, (int)-tmp_Plus, 2, (int)-tmp_Plus);
		OneMotionMovDir_Tag_1 = Motor_ID_1_Pos - tmp_Plus;
		OneMotionMovDir_Tag_2 = Motor_ID_2_Pos - tmp_Plus;
		break;
	case 'd':
		tmp_Plus = Value * Weel_plus_per_deg;
		Motor.Motor_Step_Move_Both(1, (int)tmp_Plus, 2, (int)tmp_Plus);
		OneMotionMovDir_Tag_1 = Motor_ID_1_Pos + tmp_Plus;
		OneMotionMovDir_Tag_2 = Motor_ID_2_Pos + tmp_Plus;
		break;
	}
	lastMovCmd = dir;
}

bool OneMotionMov(char dir, double Value) {
	return OneMotionMov(dir, Value, spd, acc);
}
bool OneMotionMov(char dir, double Value, double spd_input, double acc_input) {
	bool ReturnValue = false;

	int Motor_diff_1, Motor_diff_2;

	switch (OneMotionMovDir) {
	case OneMotionMovCase::WaitMotorReady:
		MotorMoveType2(0, 0);
		if ((Motor.Motor_State_Read(1) != 3) || (Motor.Motor_State_Read(2) != 3)) {
			break;
		}
		Motor.JOG_Setting(spd_input, acc_input, 1);
		Motor.JOG_Setting(spd_input, acc_input, 2);
		OneMotionMovDir = OneMotionMovCase::SendMotion;
	case OneMotionMovCase::SendMotion:
		MotorDirPlusMove(dir, Value);
		//printf("OneMotionMovCase::SendMotion.\n");
		OneMotionMovDir = OneMotionMovCase::WaitOneMotionEnd;
		break;
	case OneMotionMovCase::CheckMotionArrive:
		Motor_diff_1 = OneMotionMovDir_Tag_1 - Motor.Motor_Pos_Read(1);
		Motor_diff_2 = OneMotionMovDir_Tag_2 - Motor.Motor_Pos_Read(2);
		if ( abs(Motor_diff_1) > 100 || abs(Motor_diff_2) > 100) {
			OneMotionMovDir = OneMotionMovCase::ReSentMotion;
		}
		else {
			OneMotionMovDir = OneMotionMovCase::EndOneMotionProcess;
			//printf("OneMotionMovCase::EndOneMotionProcess.\n");
		}
		break;
	case OneMotionMovCase::ReSentMotion:
		Motor_diff_1 = OneMotionMovDir_Tag_1 - Motor.Motor_Pos_Read(1);
		Motor_diff_2 = OneMotionMovDir_Tag_2 - Motor.Motor_Pos_Read(2);
		Motor.Motor_Step_Move_Both(1, Motor_diff_1, 2, Motor_diff_2);
		lastMovCmd = MotorDirectionReturn(Motor_diff_1, Motor_diff_2);
		//printf("OneMotionMovCase::ReSentMotion.\n");
		OneMotionMovDir = OneMotionMovCase::WaitOneMotionEnd;
		break;
	case OneMotionMovCase::WaitOneMotionEnd:
		if ((Motor.Motor_State_Read(1) == 3) && (Motor.Motor_State_Read(2) == 3)) {
			OneMotionMovDir = OneMotionMovCase::CheckMotionArrive;
		}
		break;
	case OneMotionMovCase::EndOneMotionProcess:
		ReturnValue = true;
		OneMotionMovDir = 0;
		break;
	}
	return ReturnValue;

	//WaitMotorReady = 0,
	//	SendMotion,
	//	CheckMotionArrive,
	//	ReSentMotion,
	//	WaitOneMotionEnd,
	//	EndOneMotionProcess

}

void MotorMove(double lineSpd, double anguSpd) {

	double couveDifSpd = 0.075 * 5 * spd * couveRate;
	double spd_b = 0.08 * Weel_spd_meter_sec_2_RPM;

	if (lineSpd == 0 && anguSpd == 0) {
		MotorMoveControlType2('s', 0, 0);
	}
	else if (lineSpd == 0 && anguSpd != 0) {
		if (anguSpd > 0) {
			MotorMoveControlType2('a', -spd * turnRate, -spd * turnRate);
		}
		else if (anguSpd < 0) {
			MotorMoveControlType2('d', spd * turnRate, spd * turnRate);
		}
	}
	else if (lineSpd != 0 && anguSpd == 0) {
		if (lineSpd > 0) {
			MotorMoveControlType2('w', spd, -spd);
		}
		else if (lineSpd < 0) {
			MotorMoveControlType2('x', -spd_b, spd_b);
		}
	}
	else if (lineSpd != 0 && anguSpd != 0) {
		if (fabs(anguSpd) < 0.03) {
			MotorMoveControlType2('w', spd, -spd);
		}
		else if (anguSpd > 0) {
			couveRate = fabs(anguSpd);
			MotorMoveControlType2('q', spd - (couveDifSpd), -spd - (couveDifSpd));
		}
		else if (anguSpd < 0) {
			couveRate = fabs(anguSpd);
			MotorMoveControlType2('e', spd + (couveDifSpd), -spd + (couveDifSpd));
		}
	}
	Path_A_pre = anguSpd;
	Path_L_pre = lineSpd;
}
void MotorMoveType2(double lineSpd, double anguSpd) {

	double spdL = 0, spdR = 0;
	double Tran2RPM_rate = 0;
	char direct = 's';

	if (lineSpd == 0 && anguSpd == 0) {
		MotorMoveControlType2('s', 0, 0);
	}
	else if (lineSpd != 0 && anguSpd == 0) {
		Tran2RPM_rate = Weel_spd_meter_sec_2_RPM;
		spdL = lineSpd * Tran2RPM_rate;
		spdR = -lineSpd * Tran2RPM_rate;
		MotorMoveControlType2('w', spdL, spdR);
	}
	else if (lineSpd == 0 && anguSpd != 0) {
		Tran2RPM_rate = Weel_spd_rad_sec_2_RPM;
		if (anguSpd > 0) {
			direct = 'a';
		}
		else {
			direct = 'd';
		}
		spdL = -anguSpd * Tran2RPM_rate;
		spdR = -anguSpd * Tran2RPM_rate;
		MotorMoveControlType2(direct, spdL, spdR);
	}
	else if (lineSpd != 0 && anguSpd != 0) {
		Tran2RPM_rate = Weel_spd_meter_sec_2_RPM;
		double tmp_diff = CAR_WELL_to_Weel_L_meter / fabs(lineSpd / anguSpd);
		if (anguSpd > 0) {
			direct = 'q';
			spdL = lineSpd * Tran2RPM_rate * (1 - tmp_diff);
			spdR = -lineSpd * Tran2RPM_rate * (1 + tmp_diff);;
		}
		else {
			direct = 'e';
			spdL = lineSpd * Tran2RPM_rate * (1 + tmp_diff);
			spdR = -lineSpd * Tran2RPM_rate * (1 - tmp_diff);;
		}
		MotorMoveControlType2(direct, spdL, spdR);
	}
	Path_A_pre = anguSpd;
	Path_L_pre = lineSpd;

}
void MotorMoveControl(char dir) {

	if (FrontObjectClose && HitObjectStop) {
		switch (dir) {
		case 'w':
		case 'q':
		case 'e':
			dir = 's';
			break;
		}
	}
	if (BackObjectClose && HitObjectStop && (dir == 'x') ){
		dir = 's';
	}

	
	//double couveDifSpd = 0.075 * 5 * spd * couveRate;
	double couveDifSpd = 0.075 * spd * ManualCouveRate;
	double spd_b = 0.08 * Weel_spd_meter_sec_2_RPM; 
	switch (dir) {
	case 'w':
		Motor.JOG_Move(1, acc, spd);
		Motor.JOG_Move(2, acc, -spd);
		break;
	case 'x':
		Motor.JOG_Move(1, acc, -spd);
		Motor.JOG_Move(2, acc, spd);
		break;
	case 'a':
		Motor.JOG_Move(1, acc, -spd * turnRate);
		Motor.JOG_Move(2, acc, -spd * turnRate);
		break;
	case 'd':
		Motor.JOG_Move(1, acc, spd  * turnRate);
		Motor.JOG_Move(2, acc, spd  * turnRate);
		break;
	case 's':
		Motor.Motor_STOP_Group();
		break;
	case 'q':
		Motor.JOG_Move(1, acc, spd - (couveDifSpd));
		Motor.JOG_Move(2, acc, -spd - (couveDifSpd));
		break;
	case 'e':
		Motor.JOG_Move(1, acc, spd + (couveDifSpd));
		Motor.JOG_Move(2, acc, -spd + (couveDifSpd));
		break;
	case 'b':
		Motor.JOG_Move(1, acc, -spd_b);
		Motor.JOG_Move(2, acc, spd_b);
		break;
	default:
		dir = 's';
	}

	lastMovCmd = dir;

}
void MotorMoveControlType2(char dir, double spd_L, double spd_R) {

	if (FrontObjectClose) {
		switch (dir) {
		case 'w':
		case 'q':
		case 'e':
			dir = 's';
			break;
		}
	}
	if (BackObjectClose && HitObjectStop && (dir == 'x')) {
		dir = 's';
	}

	switch (dir) {
	case 'w':
	case 'a':
	case 'd':
	case 'q':
	case 'e':
		Motor.JOG_Move(1, acc, spd_L);
		Motor.JOG_Move(2, acc, spd_R);
		break;
	case 's':
		Motor.Motor_STOP_Group();
		break;
	}

	lastMovCmd = dir;

}

void SelfPosUpdate(bool speed_on, double time) {
	double p1, p2, s1, s2;
	p1 = Motor.Motor_Pos_Read(1);
	p2 = Motor.Motor_Pos_Read(2);
	Motor_Pos_Read_M1 = p1;
	Motor_Pos_Read_M2 = p2;
	if (speed_on) {
		RobotOdomInfo.GetPos(p1, p2, time);
	}
	else {
		RobotOdomInfo.GetPos(p1, p2);
	}
	Motor_Step_L = p1;
	Motor_Step_R = p2;
}
double FPS_Calculate() {

	Odom_current_time = ros::Time::now();
	double dt = (Odom_current_time - Odom_last_time).toSec();
	FPS_buffer[FPS_buffer_count % FPS_bufferSize] = dt;

	FPS_avg = 0.00000001;
	for (int i = 0; i < FPS_bufferSize; i++) {
		FPS_avg = FPS_avg + (FPS_buffer[i] * 0.1);
	}
	FPS_buffer_count++;

	Odom_last_time = Odom_current_time;
	return dt;

}
double GetTagDegDiff(cv::Point2d Tag) {
	double MoveDir = TranFunc.targetDegReturn(Tag.x, Tag.y);
	double nowD = TranFunc.TF_Pos.Degree;
	double savetmd2_1, savetmd2_2, savetmd2_work;

	savetmd2_1 = MoveDir - nowD;
	if (savetmd2_1 > 0)
		savetmd2_2 = savetmd2_1 - 360;
	else if (savetmd2_1 < 0)
		savetmd2_2 = 360 + savetmd2_1;
	else
		savetmd2_2 = 0;
	if (fabs(savetmd2_1) > fabs(savetmd2_2)) {
		savetmd2_work = savetmd2_2;
		savetmd2_2 = savetmd2_1;
		savetmd2_1 = savetmd2_work;
	}
	else
		savetmd2_work = savetmd2_1;

	return savetmd2_work;
}
double GetCircleCenterDistance(cv::Point2d Tag,double TagDegDiff) {
	cv::Point2d CircleCenter,Distance;
	double CenterDeg;
	if (TagDegDiff < 0) {
		CenterDeg = TranFunc.TF_Pos.theta - (CV_PI * 0.5);
	}
	else  {
		CenterDeg = TranFunc.TF_Pos.theta + (CV_PI * 0.5);
	}
	CircleCenter = cv::Point2d(TranFunc.TF_Pos.Pos.x + MaxTurnRadius*cos(CenterDeg), TranFunc.TF_Pos.Pos.y + MaxTurnRadius*sin(CenterDeg));
	
	
	return cv::norm(CircleCenter - Tag);
}

cv::Point2d GetPatrolPoint(int Dir) {
	return TranFunc.Img2Real(PatrolPoint[Dir % PatrolPoint.size()] - cv::Point(Map_Rect.x, Map_Rect.y));
}
cv::Point2d GetRoomPoint(int RoomID, int Step) {
	return TranFunc.Img2Real(RoomSetPoint[RoomID][Step] - cv::Point(Map_Rect.x, Map_Rect.y));
}
void GoCharegPointProcess() {

	int tmpT;
	int TmpV;
	char MovDir;

	if (CamSayWorking == 2) {
		CallCamWork = 0;
	}

	if (CamSayWorking && TellCamMotorEnd == 1) {

		if (CamMotorCmdType.size() > 0) {
			tmpT = CamMotorCmdType[CamMotorCmdDir];
			TmpV = CamMotorCmdValue[CamMotorCmdDir];

			while (tmpT == 0 || TmpV == 0) {
				CamMotorCmdDir++;
				if (CamMotorCmdDir == CamMotorCmdType.size()) {
					break;
				}
				else {
					tmpT = CamMotorCmdType[CamMotorCmdDir];
					TmpV = CamMotorCmdValue[CamMotorCmdDir];
				}
			}
			if (tmpT == 1) {
				MovDir = 0;
				if (TmpV > 0) {
					MovDir = 'w';
				}
				else if (TmpV < 0) {
					MovDir = 'x';
					TmpV = -TmpV;
				}
				if (OneMotionMov(MovDir, TmpV / 100.0, CharegProcessSpd, CharegProcessAcc) == true) {
					CamMotorCmdDir++;
				}
			}
			else if (tmpT == 2) {
				if (TmpV > 0) {
					MovDir = 'a';
				}
				else if (TmpV < 0) {
					MovDir = 'd';
					TmpV = -TmpV;
				}
				if (OneMotionMov(MovDir, TmpV / 100.0, CharegProcessSpd, CharegProcessAcc) == true) {
					CamMotorCmdDir++;
				}
			}
			else if (tmpT == 3) {
				CharegProcessSpd = TmpV;
				CamMotorCmdDir++;
			}
			else if (tmpT == 4) {
				CharegProcessAcc = TmpV;
				CamMotorCmdDir++;
			}
			else {
				CamMotorCmdDir++;
			}
		}
		if (CamMotorCmdDir_Pre != CamMotorCmdDir) {
			printf("Motor Working size = %d, D=%d_T=%d_V=%d \n", CamMotorCmdType.size(), CamMotorCmdDir, tmpT, TmpV);
			CamMotorCmdDir_Pre = CamMotorCmdDir;
		}
		if (CamMotorCmdDir >= CamMotorCmdType.size()) {
			printf("End Working D=%d_T=%d_V=%d \n", CamMotorCmdDir, tmpT, TmpV);
			TellCamMotorEnd = 2;
		}
	}
	if (TellCamMotorEnd == 2) {
		if (CamSayCmd == 0) {
			TellCamMotorEnd = 0;
		}
	}
}

void Para_2_Img() {

	std::string tmpString;
	int setposxString = 10, setposy = 20;

	cv::Mat img = Mat::zeros(cv::Size(400, 400), CV_8UC3);

	Drew_IMG(&img, "TF_x", std::to_string(1000 * TranFunc.TF_Pos.Pos.x), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "Odom.x", std::to_string(RobotOdomInfo.SelfPos.COORD.x), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "TF_y", std::to_string(1000 * TranFunc.TF_Pos.Pos.y), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "Odom.y", std::to_string(RobotOdomInfo.SelfPos.COORD.y), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "TF_deg", std::to_string(TranFunc.TF_Pos.Degree), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "Odom.D", std::to_string(RobotOdomInfo.SelfPos.Degree), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "Tag.x", std::to_string(normalMovTag.x), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "Tag.y", std::to_string(normalMovTag.y), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "RPM_1", std::to_string(Motor.Motor_Speed_RPM_Read(1)), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "RPM_2", std::to_string(Motor.Motor_Speed_RPM_Read(2)), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "acc", std::to_string(acc), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "spd", std::to_string(spd), cv::Point(setposxString + 200, setposy));
	setposy += 20;


	Drew_IMG(&img, "FPS", std::to_string(1 / FPS_avg), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "HitObjC", std::to_string(normalMovHitObjCount), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	//Drew_IMG(&img, "AutoRun", std::to_string(AutoMove), cv::Point(setposxString, setposy));
	//Drew_IMG(&img, "MBMode", std::to_string(baseControlMode), cv::Point(setposxString + 200, setposy));
	//setposy += 20;
	Drew_IMG(&img, "FObjLow", std::to_string(FrontObject_LowOnly), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "DoorOpenC", std::to_string(DoorOpenCounter), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "FObjLaser", std::to_string(FrontObject_Laser), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "DoorClosed", std::to_string(DoorClosed), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	//Drew_IMG(&img, "PathT", std::to_string((ros::Time::now() - Path_last_time).toSec()), cv::Point(setposxString, setposy));
	//Drew_IMG(&img, "MovTagN", std::to_string(patrolMovTagCount), cv::Point(setposxString + 200, setposy));
	//setposy += 20;
	Drew_IMG(&img, "ModeNow", std::to_string(ModeNow), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "ModeCC", std::to_string(ModeChangeCmd), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "DegDiff", std::to_string(DegDiff_SHOW), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "CircleRate", std::to_string(CircleRate_SHOW), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "State_1", std::to_string(Motor.Motor_State_Read(1)), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "State_2", std::to_string(Motor.Motor_State_Read(2)), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "SysWkDet", std::to_string(SystemWorkDetect), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "FObjVal", std::to_string(FrontObjectClose_Value), cv::Point(setposxString + 200, setposy));
	setposy += 20;

	Drew_IMG(&img, "WorkLpMode", std::to_string(WorkLoopSleepProcess), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "WorkLpTime", std::to_string(GetWorkLoopTime), cv::Point(setposxString + 200, setposy));
	setposy += 20;

	int input3 = 2000
		+ (int)(9 * GPIO_Control.GPIO_Read_Float(0))
		+ (int)(9 * GPIO_Control.GPIO_Read_Float(1)) * 10
		+ (int)(9 * GPIO_Control.GPIO_Read_Float(2)) * 100;
	Drew_IMG(&img, "INPUT", std::to_string(input3), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "BackObj", std::to_string(BackObjectClose), cv::Point(setposxString + 200, setposy));
	
	setposy += 20;
	Drew_IMG(&img, "Pos_1", std::to_string(Motor_Pos_Read_M1), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "Pos_2", std::to_string(Motor_Pos_Read_M2), cv::Point(setposxString + 200, setposy));
	setposy += 20;
	Drew_IMG(&img, "RoomID", std::to_string(SelRoomID), cv::Point(setposxString, setposy));
	Drew_IMG(&img, "RoomNext", std::to_string(SelRoomIDNext), cv::Point(setposxString + 200, setposy));
	setposy += 20;



	img.copyTo(Para_Monitor);

}
void Drew_IMG(cv::Mat *img, std::string Name, std::string input, cv::Point pos) {
	cv::putText(*img, Name, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 255));
	pos.x += 90;
	cv::putText(*img, input, pos, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 100, 255));

}
void ALL_Monitor_Combine() {

	if (UserGuiMode) {
		Para_2_Img();
		Rect rect1(0, 0, 400, 150);
		User_GUI.copyTo(ALL_Monitor(rect1));
		Rect rect2(0, 150, 400, 400);
		Para_Monitor.copyTo(ALL_Monitor(rect2));
		Rect rect3(0, 400, 400, 400);
		
		//cv::Mat Map_Monitor_Draw = Map_Monitor.clone();
		if (UserGuiMode == 1) {
			Laser_Monitor.copyTo(ALL_Monitor(rect3));
		}
		else if (UserGuiMode == 2) {
			CombineLaserAndMap().copyTo(ALL_Monitor(rect3));
		}
		//TranFunc.Real2Img(TranFunc.TF_Pos.Pos)

	}
	else {
		Para_2_Img();
		Rect rect1(0, 0, 400, 400);
		Para_Monitor.copyTo(ALL_Monitor(rect1));
		Rect rect2(0, 400, 400, 400);
		Laser_Monitor.copyTo(ALL_Monitor(rect2));
		//Map_Monitor_Draw = Map_Monitor.clone();	
	}
}
cv::Mat CombineLaserAndMap() {
	cv::Point Robot_P = TranFunc.Real2Img(TranFunc.TF_Pos.Pos);
	bool nonShow = false;

	cv::Point Map_MIN_Pos, Map_MAX_Pos;
	cv::Point GuiMapPosMIN = cv::Point(0, 0);
	cv::Point GuiMapPosMAX = cv::Point(399, 399);
	double MapSizeChange;
	

	cv::Mat img = Mat::zeros(cv::Size(400, 400), CV_8UC3);
	GuiMapPosMIN = cv::Point(0,0);
	GuiMapPosMAX = cv::Point(img.cols - 1, img.rows - 1);

	Map_MIN_Pos = Robot_P - cv::Point(200 * Map2LaserRate, 200 * Map2LaserRate);
	Map_MAX_Pos = Robot_P + cv::Point(200 * Map2LaserRate, 200 * Map2LaserRate);

	if (Map_MIN_Pos.x < 0) {
		MapSizeChange = (0 - Map_MIN_Pos.x) / (float)(Robot_P.x - Map_MIN_Pos.x);
		GuiMapPosMIN.x = 200 * MapSizeChange;
		Map_MIN_Pos.x = 0;
	}
	else if (Map_MIN_Pos.x >= Map_Monitor.cols) {
		nonShow = true;
	}
	if (Map_MIN_Pos.y < 0) {
		MapSizeChange = (0 - Map_MIN_Pos.y) / (float)(Robot_P.y - Map_MIN_Pos.y);
		GuiMapPosMIN.y = 200 * MapSizeChange;
		Map_MIN_Pos.y = 0;
	}
	else if (Map_MIN_Pos.y >= Map_Monitor.rows) {
		nonShow = true;
	}

	if (Map_MAX_Pos.x < 0) {
		nonShow = true;
	}
	else if (Map_MAX_Pos.x >= Map_Monitor.cols) {
		MapSizeChange = (Map_Monitor.cols - Robot_P.x) / (float)(Map_MAX_Pos.x - Robot_P.x);
		GuiMapPosMAX.x = 200 * MapSizeChange + 199;
		Map_MAX_Pos.x = Map_Monitor.cols - 1;
	}
	if (Map_MAX_Pos.y < 0) {
		nonShow = true;
	}
	else if (Map_MAX_Pos.y >= Map_Monitor.rows) {
		MapSizeChange = (Map_Monitor.rows - Robot_P.y) / (float)(Map_MAX_Pos.y - Robot_P.y);
		GuiMapPosMAX.y = 200 * MapSizeChange + 199;
		Map_MAX_Pos.y = Map_Monitor.rows - 1;
	}

	cv::Rect MapArea(Map_MIN_Pos, Map_MAX_Pos);
	cv::Rect GuiAreaArea(GuiMapPosMIN, GuiMapPosMAX);

	if (!nonShow) {
		cv::resize(Map_Monitor(MapArea), img(GuiAreaArea), img(GuiAreaArea).size());
		img = img * 0.5 + Laser_Monitor_With_deg * 0.5;
	}
	else {
		img = Laser_Monitor_With_deg;
	}
	return img;
	
	//cv::imshow("CombineLaserAndMap", img);
	
}

void SetRemoteAdress(int adr) {
	bool tmp;
	for (int i = 0; i < 8; i++) {
		if (adr & 0x01)
			tmp = true;
		else
			tmp = false;
		GPIO_Control.GPIO_Set_Value(i + 1, tmp);
		adr = adr >> 1;
	}
}
void SetCameraRec(bool SW) {
	GPIO_Control.GPIO_Set_Value(9,SW);
}
void VoicePlay(std::string VoicePath) {
	std::string VoiceTmp = (VOICE_PLAY_CMD) + VoicePath + (" &");
	system(VoiceTmp.c_str());
}
void SetRemoteSwitch(bool value) {
	GPIO_Control.GPIO_Set_Value(0, value);
}
void SystemVoiceInit(int type, std::string VoicePath, int CycleTime) {
	ProgramVoiceTime = ros::Time::now();
	ProgramVoiceType = type;
	VoicePlay(VoicePath);
}
void SystemVoicePlay(std::string VoicePath, int CycleTime) {
	if ((ros::Time::now() - ProgramVoiceTime).toSec() > CycleTime && !StopOtherVoice) {
		ProgramVoiceTime = ros::Time::now();
		VoicePlay(VoicePath);
	}
}

void VehicleStopVoiceInit(std::string VoicePath, int CycleTime) {
	VehicleStopVoiceTime = ros::Time::now();
	VoicePlay(VoicePath);
	StopOtherVoice = true;
}
void VehicleStopVoicePlay(std::string VoicePath, int CycleTime) {
	if ((ros::Time::now() - VehicleStopVoiceTime).toSec() > CycleTime) {
		VehicleStopVoiceTime = ros::Time::now();
		VoicePlay(VoicePath);
	}
}

char MotorDirectionReturn(int Step1, int Step2) {
	if (Step1 > 0) {
		if (Step2 > 0) {
			return 'd';
		}
		else if (Step2 < 0) {
			return 'w';
		}
		else {
			return 'e';
		}	
	}
	else if (Step1 < 0){
		if (Step2 > 0) {
			return 'x';
		}
		else if (Step2 < 0) {
			return 'a';
		}
		else {
			return 'a';
		}
	}
	else {
		if (Step2 > 0) {
			return 'd';
		}
		else if (Step2 < 0) {
			return 'q';
		}
		else {
			return 's';
		}
	}
}

void SetSystemInitStep(int step) {
	SystemWorkDetect = SystemWorkDetect | (0x01 << step);
	if (SystemWorkDetect_Print != SystemWorkDetect) {
		printf("SystemWorkDetect Code = %d\n", SystemWorkDetect);
		SystemWorkDetect_Print = SystemWorkDetect;
	}
	
}
void ETHER_RECV(void) {
	//printf("ETHER_RECV\n");
	EtherS.handle_udp_msg();
}

bool SetGoFrontPos(int RoomID) {
	GoFrontDoorPoint.clear();

	cv::Point Start_P = TranFunc.Real2Img(TranFunc.TF_Pos.Pos);
	cv::Point End_P = RoomSetPoint[RoomID][0] - cv::Point(Map_Rect.x, Map_Rect.y);
	
	if ( GetGobalPathFunction(TranFunc.Img2Real(End_P)) == false ) {
		return false;
	}
	GetGobalPathDir = 0;
	printf("SetGoFrontPos ,Start_P (%d,%d),End_P(%d,%d) \n", Start_P.x, Start_P.y, End_P.x, End_P.y);

	if (Map_Router.AllMapCheck()) {
		Map_Router.Refresh();
		if (GetGobalPathTest) {
			Map_Router.FindRouteProcess_Block_Only(Start_P, End_P, golbalPath, golbalPath.size());
		}
		else {
			Map_Router.FindRouteProcess_Block_Auto(Start_P, End_P);
		}
	}
	printf("SetGoFrontPos Get Route Size = %d \n", Map_Router.RouteNew.size());

	cv::Point2d Tag_Pos;

	for (int i = Map_Router.RouteNew.size() - 2; i >= 0; i--) {
		Tag_Pos = TranFunc.Img2Real(Map_Router.RouteNew[i]);
		GoFrontDoorPoint.push_back(Tag_Pos);
		printf("%d _ Pos(%.2f,%.2f)\n", GoFrontDoorPoint.size() - 1, Tag_Pos.x, Tag_Pos.y);
	}
	return true;

}
bool SetGoStartPos() {
	GoStartPosPoint.clear();

	cv::Point Start_P = TranFunc.Real2Img(TranFunc.TF_Pos.Pos);
	cv::Point End_P = PatrolPoint[0] - cv::Point(Map_Rect.x, Map_Rect.y);

	if (GetGobalPathFunction(TranFunc.Img2Real(End_P)) == false) {
		return false;
	}
	GetGobalPathDir = 0;

	printf("GoStartPosPoint ,Start_P (%d,%d),End_P(%d,%d) \n", Start_P.x, Start_P.y, End_P.x, End_P.y);

	if (Map_Router.AllMapCheck()) {
		Map_Router.Refresh();
		if (GetGobalPathTest) {
			Map_Router.FindRouteProcess_Block_Only(Start_P, End_P, golbalPath, golbalPath.size());
		}
		else {
			Map_Router.FindRouteProcess_Block_Auto(Start_P, End_P);
		}
	}

	cv::Point2d Tag_Pos;
	printf("SetGoStartPos Get Route Size = %d \n", Map_Router.RouteNew.size());

	for (int i = Map_Router.RouteNew.size() - 2; i >= 0; i--) {
		Tag_Pos = TranFunc.Img2Real(Map_Router.RouteNew[i]);
		GoStartPosPoint.push_back(Tag_Pos);
		printf("%d _ Pos(%.2f,%.2f)\n", GoStartPosPoint.size() - 1, Tag_Pos.x, Tag_Pos.y);
	}
	return true;
}
bool SetGoChargePos() {
	Go_ChargePoint.clear();

	cv::Point Start_P = TranFunc.Real2Img(TranFunc.TF_Pos.Pos);
	cv::Point End_P = ChargePoint[0] - cv::Point(Map_Rect.x, Map_Rect.y);

	if (GetGobalPathFunction(TranFunc.Img2Real(End_P)) == false) {
		return false;
	}
	GetGobalPathDir = 0;

	printf("SetGoChargePos ,Start_P (%d,%d),End_P(%d,%d) \n", Start_P.x, Start_P.y, End_P.x, End_P.y);

	if (Map_Router.AllMapCheck()) {
		Map_Router.Refresh();
		if (GetGobalPathTest) {
			Map_Router.FindRouteProcess_Block_Only(Start_P, End_P, golbalPath, golbalPath.size());
		}
		else {
			Map_Router.FindRouteProcess_Block_Auto(Start_P, End_P);
		}
	}
	printf("SetGoChargePos Get Route Size = %d \n", Map_Router.RouteNew.size());

	cv::Point2d Tag_Pos;
	for (int i = Map_Router.RouteNew.size() - 2; i >= 0; i--) {
		Tag_Pos = TranFunc.Img2Real(Map_Router.RouteNew[i]);
		Go_ChargePoint.push_back(Tag_Pos);
		printf("%d _ Pos(%.2f,%.2f)\n", Go_ChargePoint.size() - 1, Tag_Pos.x, Tag_Pos.y);
	}
	return true;
}
bool GetGobalPathFunction(cv::Point2d Tag) {

	if (GetGobalPathTest == false) {
		return true;
	}

	bool Result = false;
	double MoveDirTheta = 0.001 * CV_PI / 180.0;
	double re_w = cos(MoveDirTheta / 2.0);
	double re_z = sin(MoveDirTheta / 2.0);
		
	switch (GetGobalPathDir) {
	case GetGobalPathCase::PublishPathGoal:

		if (re_w > 0 && re_z < 0 && re_w < 0.5) {
			re_w = -re_w;
			re_z = -re_z;
		}
		BaseMovgoal.pose.position.x = Tag.x;
		BaseMovgoal.pose.position.y = Tag.y;
		BaseMovgoal.pose.position.z = 0;
		BaseMovgoal.pose.orientation.x = 0;
		BaseMovgoal.pose.orientation.y = 0;
		BaseMovgoal.pose.orientation.w = re_w;
		BaseMovgoal.pose.orientation.z = re_z;

		goal_pub_switch = true;
		get_path_return = false;

		GetGobalPathDir = GetGobalPathCase::WaitPathGoalCallback;
		break;
	case GetGobalPathCase::WaitPathGoalCallback:
		if (goal_pub_switch == false && get_path_return) {
			GetGobalPathDir = GetGobalPathCase::WaitCancellGoal;
			goal_cancel_pub = true;
			get_path_return = false;
		}
		break;
	case GetGobalPathCase::WaitCancellGoal:
		if (goal_cancel_pub == false) {
			GetGobalPathDir = GetGobalPathCase::EndGetGobalPathProcess;
		}
		break;
	case GetGobalPathCase::EndGetGobalPathProcess:
		Result = true;
		break;
	}
	
	return Result;

}
//////Send and receve data function
void Node_Publisher() {

	ros::NodeHandle n;
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 10);
	ros::Publisher cancel_pub = n.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1);
	ros::Publisher camTrac_pub = n.advertise<std_msgs::String>("msg_R_to_C", 10);

	tf::TransformBroadcaster odom_broadcaster;

	odometryPoint = RobotOdomInfo.SelfPos;

	double x = 0.0;
	double y = 0.0;
	double th = 0.0;

	double vx = 0.0;
	double vy = -0.0;
	double vth = 0.0;

	double dt;
	double delta_x;
	double delta_y;
	double delta_th;

	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate loop_rate(5);
	SetSystemInitStep(SystemInitStep::fu_Node_Publisher);
	while (ros::ok())
	{
		
		ros::spinOnce();

		current_time = ros::Time::now();

		////Odom publish
		//if (true) {
			//compute odometry in a typical way given the velocities of the robot
		dt = (current_time - last_time).toSec();
		delta_x = RobotOdomInfo.SelfPos.COORD.x - odometryPoint.COORD.x;
		delta_y = RobotOdomInfo.SelfPos.COORD.y - odometryPoint.COORD.y;
		delta_th = (RobotOdomInfo.SelfPos.Degree - odometryPoint.Degree);

		odometryPoint = RobotOdomInfo.SelfPos;
		delta_x = delta_x / 1000.0;
		delta_y = delta_y / 1000.0;

		if (delta_th >= 180) {
			delta_th -= 360;
		}
		else if (delta_th <= -180) {
			delta_th += 360;
		}
		delta_th = CV_PI * delta_th / 180.0;

		x += delta_x;
		y += delta_y;
		th += delta_th;
		vx = delta_x / dt;
		vy = delta_y / dt;
		vth = delta_th / dt;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

		//publish the message
		odom_pub.publish(odom);

		//}
		
		////goal publish
		if (goal_pub_switch) {
			geometry_msgs::PoseStamped goal = BaseMovgoal;
			goal.header.stamp = current_time;
			goal.header.frame_id = "map";

			goal_pub.publish(goal);

			printf("GOAL PUB = (%.1f,%.1f,.1%f),(%.1f,%.1f,%.1f,%.1f)\n",
				goal.pose.position.x,
				goal.pose.position.y,
				goal.pose.position.z,
				goal.pose.orientation.x,
				goal.pose.orientation.y,
				goal.pose.orientation.w,
				goal.pose.orientation.z
			);

			goal_pub_switch = false;
		}
		////goal cancel
		if (goal_cancel_pub) {
			actionlib_msgs::GoalID first_goal;
			cancel_pub.publish(first_goal);
			goal_cancel_pub = false;
			printf("GOAL cancel \n");

		}
		if (cam_Trac_Working) {
			std_msgs::String str_msg;
			str_msg.data = "0000";
			int stamp = (int)current_time.toSec() / 10;
			str_msg.data[3] = ((stamp % 26) + 'A') & 0x0ff;
			str_msg.data[0] = (CallCamWork + '0') & 0x0ff;
			str_msg.data[1] = (TellCamMotorEnd + '0') & 0x0ff;
			camTrac_pub.publish(str_msg);
		}

		last_time = current_time;
		loop_rate.sleep();
	}
}
void Node_Subscriber() {
	SetSystemInitStep(SystemInitStep::fu_Node_Subscriber);
	ros::NodeHandle n;

	ros::Subscriber laser_sub = n.subscribe("scan", 10, scanCallback);
	ros::Subscriber map_sub = n.subscribe("map", 1, mapCallback);
	ros::Subscriber vel_sub = n.subscribe("cmd_vel", 1, velCallback);
	ros::Subscriber path_sub = n.subscribe("move_base/NavfnROS/plan", 1, pathCallback);
	ros::Subscriber localPath_sub = n.subscribe("move_base/TrajectoryPlannerROS/local_plan", 1, localPathCallback);
	ros::Subscriber camTracsub = n.subscribe("msg_C_to_R", 1000, CamTracCallback);

	ros::Rate loop_rate(10);

	printf("Node_Subscriber Working\n");

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	printf("Node_Subscriber End %d  \n", ros::ok());

}
void velCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	g_vel = cmd_vel->linear.x;
	t_vel = cmd_vel->angular.z;
	Path_last_time = ros::Time::now();

	Path_L_buffer[Path_buffer_count % Path_bufferSize] = g_vel;
	Path_A_buffer[Path_buffer_count % Path_bufferSize] = t_vel;

	double tmpA = 0, tmpL = 0;

	for (int i = 0; i < Path_bufferSize; i++) {
		tmpL += (Path_L_buffer[i]);
		tmpA += (Path_A_buffer[i]);
	}

	Path_A_avg = tmpA / Path_bufferSize;
	Path_L_avg = tmpL / Path_bufferSize;

	Path_buffer_count++;
}
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	SetSystemInitStep(SystemInitStep::fu_scanCallback);
	int count = scan->scan_time / scan->time_increment;

	cv::Point2f point_tmp, center;
	cv::Point2f paintP;
	double angle , fabsPointY;
	float rawData, maxData = 0, minData = 100;
	Mat img = Mat::zeros(cv::Size(LaserWindowsSize, LaserWindowsSize), CV_8UC3);

	Mat img_with_deg = Mat::zeros(cv::Size(LaserWindowsSize, LaserWindowsSize), CV_8UC3);
	center.x = LaserWindowsLenth;
	center.y = LaserWindowsLenth;

	bool ThingIsClose_Front = false;
	bool ThingIsClose_Front_L = false;
	bool ThingIsClose_Front_R = false;
	bool ThingIsClose_Back = false;

	double ThingIsClose_Front_Value = FrontSafeDistance + 0.1;
	bool ScanDoorIsClose = false;

	for (unsigned int i = 0; i < count; ++i)
	{
		rawData = scan->ranges[i];
		if (rawData > maxData && rawData < 30)
			maxData = rawData;
		if (rawData < minData)
			minData = rawData;
	}

	for (int i = 0; i < count; i++) {
		angle = scan->angle_min + scan->angle_increment * i ;
		rawData = scan->ranges[i];
		point_tmp.x = (rawData * cos(angle));
		point_tmp.y = (rawData * sin(angle));
		fabsPointY = fabs(point_tmp.y);
		if (rawData > 0.200) {		
			//Object Detect
			if (fabsPointY < SideSafeDistance) {
				if (point_tmp.x > 0  && point_tmp.x < FrontSafeDistance ) {
					if ((ThingIsClose_Front_Value == 0) || (ThingIsClose_Front_Value > point_tmp.x)) {
						ThingIsClose_Front_Value = point_tmp.x;
					}
					ThingIsClose_Front = true;
					if (point_tmp.y > 0) {
						ThingIsClose_Front_L = true;
					}
					else {
						ThingIsClose_Front_R = true;
					}
				}
				else if (point_tmp.x < 0 && point_tmp.x  > -BackSafeDistance) {
					ThingIsClose_Back = true;
				}
			}
			//Door Detect
			if (fabsPointY < DoorOpenWidth_half && point_tmp.x > 0 && point_tmp.x < DoorOpenDistance ) {
				ScanDoorIsClose = true;
			}
			paintP = center + (LaserWindowsLenth * point_tmp / maxData);
			paintP.y = LaserWindowsSize - paintP.y;
			cv::circle(img, paintP, 1, cv::Scalar(0, 0, 200), CV_FILLED);
		}
		else {
			paintP = center + (LaserWindowsLenth * point_tmp / maxData);
			paintP.y = LaserWindowsSize - paintP.y;
			cv::circle(img, paintP, 1, cv::Scalar(0, 50, 150), CV_FILLED);
		}
	}
	
	cv::circle(img, center, 1, cv::Scalar(0, 100, 200), CV_FILLED);

	if (UserGuiMode == 2) {
		for (int i = 0; i < count; i++) {
			angle = (scan->angle_min) + (scan->angle_increment * i) + (TranFunc.TF_Pos.theta);
			rawData = scan->ranges[i];
			point_tmp.x = (rawData * cos(angle));
			point_tmp.y = (rawData * sin(angle));
			if (rawData > 0.200) {
				paintP = center + (LaserWindowsLenth * point_tmp / maxData);
				paintP.y = LaserWindowsSize - paintP.y;
				cv::circle(img_with_deg, paintP, 1, cv::Scalar(0, 0, 200), CV_FILLED);
			}
			else {
				paintP = center + (LaserWindowsLenth * point_tmp / maxData);
				paintP.y = LaserWindowsSize - paintP.y;
				cv::circle(img_with_deg, paintP, 1, cv::Scalar(0, 50, 150), CV_FILLED);
			}
		}
		cv::circle(img_with_deg, center, 1, cv::Scalar(0, 100, 200), CV_FILLED);
	}

	
	FrontObject_Laser = ThingIsClose_Front;

	if (LowSensorInstall) {
		if (GPIO_Control.GPIO_Read_Float(2) >= 1) {
			FrontObject_LowOnly = false;
		}
		else {
			FrontObject_LowOnly = true;
		}
	}
	else {
		FrontObject_LowOnly = false;
	}


	if (FrontObject_LowOnly || FrontObject_Laser) {
		FrontObjectClose = true;
	}
	else {
		FrontObjectClose = false;
	}


	FrontObjectClose_L = ThingIsClose_Front_L;
	FrontObjectClose_R = ThingIsClose_Front_R;

	if (ThingIsClose_Front_Value < FrontSafeDistance) {
	FrontObjectClose_Value = ThingIsClose_Front_Value;
	}
	else if (FrontObject_LowOnly && !FrontObject_Laser) {
		FrontObjectClose_Value = FrontSafeDistance * 0.985;
	}
	else{
		FrontObjectClose_Value = FrontSafeDistance * 0.8;
	}

	BackObjectClose = ThingIsClose_Back;
	DoorClosed = ScanDoorIsClose;

	mutex.lock();
	img.copyTo(Laser_Monitor);
	if (UserGuiMode == 2) {
		img_with_deg.copyTo(Laser_Monitor_With_deg);
		Map2LaserRate = maxData / LaserWindowsMeter;
	}
	mutex.unlock();

}
void mapCallback(const nav_msgs::OccupancyGridConstPtr &map) {
	SetSystemInitStep(SystemInitStep::fu_mapCallback);
	geometry_msgs::Quaternion orientation = map->info.origin.orientation;
	double yaw, pitch, roll;
	tf::Matrix3x3 mat(tf::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
	mat.getEulerYPR(yaw, pitch, roll);
	double map_theta = yaw;
	//[1455208385.821892180]: Received a 2048 X 2048 map @ 0.050 m / pix_ - 51.224998_ - 51.224998_0.000000

	ROS_INFO("Received a %d X %d map @ %.3f m/pix_%f_%f_%f",
		map->info.width,
		map->info.height,
		map->info.resolution,
		map->info.origin.position.x,
		map->info.origin.position.y,
		map_theta
	);

	Mat img = Mat::zeros(cv::Size(map->info.width, map->info.height), CV_8UC1);

	cv::Point MinPos, MaxPos;

	MinPos.x = map->info.width / 2;
	MinPos.y = map->info.height / 2;
	MaxPos = MinPos;

	for (unsigned int y = 0; y < map->info.height; y++) {
		for (unsigned int x = 0; x < map->info.width; x++) {
			unsigned int i = x + (map->info.height - y - 1) * map->info.width;
			int intensity = 205;
			if (map->data[i] >= 0 && map->data[i] <= 100) {
				intensity = round((float)(100.0 - map->data[i])*2.55);

				if (MinPos.x > x)
					MinPos.x = x;
				if (MinPos.y > y)
					MinPos.y = y;
				if (MaxPos.x < x)
					MaxPos.x = x;
				if (MaxPos.y < y)
					MaxPos.y = y;

			}
			img.at<unsigned char>(y, x) = intensity;
		}
	}

	int width = MaxPos.x - MinPos.x;
	int height = MaxPos.y - MinPos.y;
	cv::Rect rect1(MinPos, MaxPos);

	cv::Mat img_out;
	bool img_out_Work = false;
	if (width == 0 || height == 0) {
		img_out = Mat::zeros(cv::Size(300, 300), CV_8UC1);
	}
	else {
		img(rect1).copyTo(img_out);
		img_out_Work = true;
	}
	Map_Rect = rect1;

	if (img_out_Work) {
		mutex.lock();

		cv::cvtColor(img_out, Map_Monitor, CV_GRAY2BGR);

		Map_Router.MapInit(&Map_Monitor);
		Map_Router.GetMapRect(Map_Rect);
		mutex.unlock();

	}
	TranFunc.PosInit(MaxPos, MinPos, img.size(), map->info.resolution);
	printf("Get Map Min=(%d,%d)_Max(%d,%d)_width(%d),height(%d)\n", MinPos.x, MinPos.y, MaxPos.x, MaxPos.y, width, height);



}
void pathCallback(const nav_msgs::Path::ConstPtr& path) {
	golbalPath.clear();
	cv::Point tmp, tmp_Pre;

	double x = 0, y = 0;
	//double x=0, x_pre=0;
	//double y=0, y_pre=0;
	//double w = path->poses[0.5*size].pose.orientation.w;
	//double z = path->poses[0.5*size].pose.orientation.z;

	int size = path->poses.size();

	for (int i = 0; i < size; i++) {
		x = path->poses[i].pose.position.x;
		y = path->poses[i].pose.position.y;

		tmp = TranFunc.Real2Img(x, y);
		if (i == 0 || (tmp_Pre.x != tmp.x || tmp_Pre.x != tmp.y)) {
			golbalPath.push_back(tmp);
		}
		tmp_Pre = tmp;
	}

	golbalFinalPoint = cv::Point(x, y);
	get_path_return = true;
}
void localPathCallback(const nav_msgs::Path::ConstPtr& path) {
	localPath.clear();
	cv::Point tmp, tmp_Pre;
	cv::Point2d tmp2f;

	double x = 0, y = 0;

	int size = path->poses.size();

	for (int i = 0; i < size; i++) {
		x = path->poses[i].pose.position.x;
		y = path->poses[i].pose.position.y;

		tmp = TranFunc.Real2Img(x, y);
		if (i == 0 || (tmp_Pre.x != tmp.x || tmp_Pre.x != tmp.y)) {
			localPath.push_back(tmp);
		}
		tmp_Pre = tmp;

	}
	if (size > 0) {
		movTag_buffer[movTag_buffer_count % movTag_bufferSize] = cv::Point2d(x, y);
		movTag_buffer_count++;
	}

	tmp2f = cv::Point2d(0, 0);
	for (int i = 0; i < movTag_bufferSize; i++) {
		tmp2f += movTag_buffer[i];
	}
	localMovTag_avg = tmp2f / 5;


}
void CamTracCallback(const std_msgs::Int32MultiArray::ConstPtr& array) {
	int i = 0;
	int cmdbuff = 0;

	for (std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		if (i == 0) {
			CamSayWorking = *it;
		}
		else if (i == 1) {
			CamSayCmd = *it;
			if (CamSayCmd && TellCamMotorEnd == 0) {
				CamMotorCmdValue.clear();
				CamMotorCmdType.clear();
				CamMotorCmdDir_Pre = -1;
				CamMotorCmdDir = 0;
			}
		}
		else if (TellCamMotorEnd == 0 && CamSayCmd && i > 1) {
			cmdbuff = *it;
			CamMotorCmdType.push_back(cmdbuff);
			it++;
			i++;
			cmdbuff = *it;
			CamMotorCmdValue.push_back(cmdbuff);
		}
		i++;
	}

	if (TellCamMotorEnd == 0 && CamSayCmd) {
		std::cout << "Get_";
		for (int i = 0; i < CamMotorCmdValue.size(); i++) {
			std::cout << i << "=(" << CamMotorCmdType[i] << "," << CamMotorCmdValue[i] << ")_";
		}
		TellCamMotorEnd = 1;
		printf("\n");
	}


	return;
}
