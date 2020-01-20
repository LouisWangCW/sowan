#include <ros/ros.h>
#include <stdio.h>
#include <vector>
#include <iostream>
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
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <ecl/threads.hpp>
#include <ecl/time/sleep.hpp>
#include "ether_server.hpp"
#include "Car_Setting_Pos.h"
#include "RS485_TECO_Motor.h"
#include "ether_server.hpp"
#include "Car_Setting_Pos.h"

using namespace cv;

void main_init();
void main_function();
void SowanParaSetting();
int main(int argc, char **argv);

void MotorMove(double lineSpd, double anguSpd);
void MotorMoveType2(double lineSpd, double anguSpd);
void MotorMoveControl(char dir);
void MotorMoveControlType2(char dir, double spd_L, double spd_R);
void MotorDirPlusMove(char dir, double Value);

int normal_Mov_Fuction(cv::Point2d Tag, double ArrRange, double TurnNeedDeg, double CircleNeedDeg, double MoveDis, double DegDiff);
int normalMovHitObjFuc(int type, cv::Point2d Tag, double DegDiff);
int normal_Mov(cv::Point2d Tag);
int normal_Mov_Fu(cv::Point2d Tag);
int Base_Mov(cv::Point2d Tag);
bool FaceTag(cv::Point2d Tag);

bool OneMotionMov(char dir, double Value);
bool OneMotionMov(char dir, double Value, double spd_input, double acc_input);

void ModeControlSystem();
void AutoMoveFunction_Test();
bool PatrolMoveFunction();
bool GoStartPosFunction();
bool WaitTimeFunction();
void LoopWorkTimeCounter();
int InterruptGotoRoom(int RoomID);
int InterruptGotoCharge();
bool StandbyFunction();
void GoCharegPointProcess();

void GUI_control(char ttt);
void User_GUI_control(int ttt);
void Para_2_Img();
void Drew_IMG(cv::Mat *img, std::string Name, std::string input, cv::Point pos);
void ALL_Monitor_Combine();
cv::Mat CombineLaserAndMap();
void mapOnMouse(int Event, int x, int y, int flags, void* param);
void switchOnMouse(int Event, int x, int y, int flags, void* param);

void SetSystemInitStep(int step);
void SelfPosUpdate(bool speed_on, double time);
void ETHER_RECV(void);
void SetRemoteAdress(int adr);
void SetRemoteSwitch(bool value);
void SetCameraRec(bool SW);
cv::Point2d GetPatrolPoint(int Dir);
cv::Point2d GetRoomPoint(int RoomID, int Step);
double FPS_Calculate();
double GetTagDegDiff(cv::Point2d Tag);
double GetCircleCenterDistance(cv::Point2d Tag, double TagDegDiff);
char MotorDirectionReturn(int Step1, int Step2);

bool SetGoChargePos();
bool SetGoStartPos();
bool SetGoFrontPos(int RoomID);
bool GetGobalPathFunction(cv::Point2d Tag);

void VoicePlay(std::string VoicePath);
void SystemVoicePlay(std::string VoicePath, int CycleTime);
void SystemVoiceInit(int type, std::string VoicePath, int CycleTime);
void VehicleStopVoicePlay(std::string VoicePath, int CycleTime);
void VehicleStopVoiceInit(std::string VoicePath, int CycleTime);

void Node_Publisher();
void Node_Subscriber();
void velCallback(const geometry_msgs::Twist::ConstPtr& vel);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
void mapCallback(const nav_msgs::OccupancyGridConstPtr &map);
void pathCallback(const nav_msgs::Path::ConstPtr& path);
void localPathCallback(const nav_msgs::Path::ConstPtr& path);
void CamTracCallback(const std_msgs::Int32MultiArray::ConstPtr& array);

enum SystemInitStep {
	fu_main = 0,
	fu_main_init,
	fu_SowanParaSetting,
	fu_Node_Publisher,
	fu_Node_Subscriber,
	fu_scanCallback,
	fu_mapCallback,
	fu_pathCallback,
	fu_localPathCallback,
	fu_CamTracCallback
};

const char *SystemModeString[] = {
	"StandbyMode",
	"StandbyWithTimeMode",
	"PatrolMode",
	"GoRoomMode",
	"GoChargeMode",
	"GoStartPosMode",
	"LoopWorkSleepMode",
};

enum SystemMode {
	StandbyMode = 0,
	StandbyWithTimeMode,
	PatrolMode,
	GoRoomMode,
	GoChargeMode,
	GoStartPosMode,
	LoopWorkSleepMode,
};

enum BaseMovCase {
	SetTag = 0,
	WaitGoalPub,
	GetVelThenMov,
	ClearDir
};

enum normalMovAdvCase {
	MoveByNormal = 0,
	NormalFailBackMov,
	WaitNF_BackMovEnd,
	MoveByBaseMov,
	MoveEnd
};

enum GotoRoomCase {
	WaitRobotReady = 0,
	GoToDoorFrontPos,
	FaceToDoor,
	SentDoorOpenSignal,
	WatiDoorOpen,
	GoToInsideDoorPos,
	GoToStandbyPos,
	FaceToBed,
	WaitUser,
	FaceInsideDoor,
	GoToInsideDoorPos_II,
	GoToDoorFrontPos_II,
	WaitRoomCaseEnd,
	EndRoomProcess
};

const char *GotoRoomCaseString[] = {
	"WaitRobotReady",
	"GoToDoorFrontPos",
	"FaceToDoor",
	"SentDoorOpenSignal",
	"WatiDoorOpen",
	"GoToInsideDoorPos",
	"GoToStandbyPos",
	"FaceToBed",
	"WaitUser",
	"FaceInsideDoor",
	"GoToInsideDoorPos_II",
	"GoToDoorFrontPos_II",
	"WaitRoomCaseEnd",
	"EndRoomProcess"
};

enum GotoChargPosCase {
	Charge_WaitRobotReady = 0,
	GoToChargPos,
	FaceToChargPos,
	CallCamToWork,
	WaitCamWorkEnd,
	WaitChargeTimeEnd,
	GetDownChargePlatform,
	EndChargPosProcess
};
const char *GotoChargPosString[] = {
	"Charge_WaitRobotReady",
	"GoToChargPos",
	"FaceToChargPos",
	"CallCamToWork",
	"WaitCamWorkEnd",
	"WaitChargeTimeEnd",
	"GetDownChargePlatform",
	"EndChargPosProcess",
};

enum OneMotionMovCase {
	WaitMotorReady = 0,
	SendMotion,
	CheckMotionArrive,
	ReSentMotion,
	WaitOneMotionEnd,
	EndOneMotionProcess
};

enum GetGobalPathCase {
	PublishPathGoal = 0,
	WaitPathGoalCallback,
	WaitCancellGoal,
	EndGetGobalPathProcess
};
