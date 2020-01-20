
///Louis ver.2018/03/07 1147
///Louis ver.2019/01/25 1726

#ifndef __CAR_SETTING__
#define __CAR_SETTING__

#include <math.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <fstream>

#include "def.hpp"
#define P2P_THRESHOLD 0.01

#define Int32 int



//public class SemsorStopDataSave {
//public:
//	char WeelDir = 'S';
//	int WeelSpeed = 0;
//	bool Worked = false;
//	void clear(void) {
//		WeelDir = 'S';
//		WeelSpeed = 0;
//		Worked = false;
//	}
//	void save(char dir, int speed) {
//		if (dir != 'S') {
//			Worked = true;
//			WeelDir = dir;
//			WeelSpeed = speed;
//		}
//
//	}
//};

class FileRead{

	std::fstream file;
	char buffer[200];
public:
	FileRead() {
		
		file.open("/home/ubuntu/sowan.txt", std::ios::in);
		CloseFile();
	}
	~FileRead() {

	}
	void CloseFile() {

		if (!file) {
			std::cout << "Reaf Fail" << std::endl;
		}
		else {
			std::cout << "Reaf Success" << std::endl;
			file.read(buffer, sizeof(buffer));
			std::cout << buffer << std::endl;
			file.close();
		}
	}
};





class MotorWorkState {
public:
	uchar MoveDir;
	int MotorSpeed;
	bool JogWork;
	int JogValue;
	MotorWorkState() {
		MoveDir = 'S';
		MotorSpeed = 0;
		JogWork = false;
		JogValue = 0;
	}

	void JogWorkSave(char Dir, int Speed, int value) {
		MoveDir = Dir;
		JogValue = value;
		MotorSpeed = Speed;
		JogWork = true;
	}
	void RunWorkSave(char Dir, int Speed) {
		MoveDir = Dir;
		JogValue = -1;
		MotorSpeed = Speed;
		JogWork = false;
	}
	void WorkStop() {
		MoveDir = 'S';
		MotorSpeed = 0;
		JogWork = false;
	}

};

class PosTran {
	struct tf_Pos {
	public:
		cv::Point2f Pos;
		double Degree, theta;
	};

private:
	cv::Point MAX_P, MIN_P, ORI_P;
	cv::Size MpaSize;

	float MatScale;
	bool init_end = false;

private:
	double x, y, z, w;
public:
	tf_Pos TF_Pos;

public:
	void TF_Pos_update(double tf_x, double tf_y, double tf_z, double tf_w) {
		x = tf_x;
		y = tf_y;
		w = tf_z;
		z = tf_w;
		TF_Pos.Pos.x = x;
		TF_Pos.Pos.y = y;

		if (w < 0) {
			TF_Pos.theta = 2 * asin(-z);
		}
		else {
			TF_Pos.theta = 2 * asin(z);
		}
		TF_Pos.Degree = 180 * TF_Pos.theta / CV_PI;
		if (TF_Pos.Degree < 0) {
			TF_Pos.Degree += 360;
		}
	}

public:
	PosTran() {};
	~PosTran() {};
public:
	void PosInit(cv::Point Max_P, cv::Point Min_P, cv::Size Map_Size, float Scale) {
		MAX_P = Max_P;
		MIN_P = Min_P;
		MpaSize.width = Max_P.x - Min_P.x;
		MpaSize.height = Max_P.y - Min_P.y;

		ORI_P.x = (Map_Size.width / 2) - Min_P.x;
		ORI_P.y = (Map_Size.height / 2) - Min_P.y;
		MatScale = Scale;

		TF_Pos_update(0, 0, 0, 0);

		init_end = true;

	}
	cv::Point Robot2Img(cv::Point Rob_P) {
		cv::Point Img_P;
		Img_P.x = ORI_P.x + Rob_P.x;
		Img_P.y = ORI_P.y - Rob_P.y;
		return Img_P;
	}
	cv::Point Img2Robot(cv::Point Img_P) {
		cv::Point Rob_P;
		Rob_P.x = Img_P.x - ORI_P.x;
		Rob_P.y = -Img_P.y + ORI_P.y;
		return Rob_P;
	}
	cv::Point Real2Robot(float x, float y) {
		cv::Point Rob_P;
		Rob_P.x = x / MatScale;
		Rob_P.y = y / MatScale;
		return Rob_P;
	}
	cv::Point Real2Img(float x, float y) {
		cv::Point Rob_P, Img_P;
		Rob_P.x = x / MatScale;
		Rob_P.y = y / MatScale;
		Img_P.x = ORI_P.x + Rob_P.x;
		Img_P.y = ORI_P.y - Rob_P.y;
		return Img_P;
	}
	cv::Point Real2Img(cv::Point2f Real_Pos) {
		return Real2Img(Real_Pos.x, Real_Pos.y);
	}
	cv::Point2f Img2Real(cv::Point Img_P) {
		cv::Point Rob_P;
		cv::Point2f real;
		Rob_P.x = Img_P.x - ORI_P.x;
		Rob_P.y = -Img_P.y + ORI_P.y;
		real.x = Rob_P.x * MatScale;
		real.y = Rob_P.y * MatScale;
		return real;
	}
	double targetRadReturn(double self_x, double self_y, double tag_x, double tag_y) {

		double deltaX = tag_x - self_x;
		double deltaY = tag_y - self_y;
		//double deltaX = tag_x;
		//double deltaY = tag_y;

		double MoveDir = atan(deltaY / deltaX);

		if (deltaX < 0)
			MoveDir += M_PI;
		while (MoveDir < 0)
			MoveDir += (2 * M_PI);
		while (MoveDir > (2 * M_PI))
			MoveDir -= (2 * M_PI);

		return MoveDir;
	}
	double targetDegReturn(double self_x, double self_y, double tag_x, double tag_y) {
		return (targetRadReturn(self_x, self_y, tag_x, tag_y) * 180 / M_PI);
	}
	double targetRadReturn(double tag_x, double tag_y) {
		return targetRadReturn(TF_Pos.Pos.x, TF_Pos.Pos.y, tag_x, tag_y);
	}
	double targetDegReturn(double tag_x, double tag_y) {
		return (targetRadReturn(TF_Pos.Pos.x, TF_Pos.Pos.y, tag_x, tag_y) * 180 / M_PI);
	}
	double targetDistanceReturn(double self_x, double self_y, double tag_x, double tag_y) {
		double deltaX = tag_x - self_x;
		double deltaY = tag_y - self_y;
		//double deltaX = tag_x;
		//double deltaY = tag_y;
		double distance = pow(pow(deltaX, 2) + pow(deltaY, 2), 0.5);
		return distance;
	}
	double targetDistanceReturn(double tag_x, double tag_y) {
		return targetDistanceReturn(TF_Pos.Pos.x, TF_Pos.Pos.y, tag_x, tag_y);
	}


	bool InitFinish() {
		return init_end;
	}

};


class CarPosCalsulate {
public:
	CarPosCalsulate() {};
	~CarPosCalsulate() {};
	///Louis ver.2017/11/01
public:
	struct Position
	{
		cv::Point2f COORD;
		double Degree = 0;
		bool passed = false;
	};
	struct Command
	{
		char Action = 'S';
		double Value = 0;
	};


private:
	int steptmpl = 0;
	int steptmpr = 0;
	double changerate;
	int MotorStepL;
	int MotorStepR;
	int steptmpL = 0;
	int steptmpR = 0;
private:
	Int32 Speed_L_Pre;
	Int32 Speed_R_Pre;
	int ERROR_Code_SAVE;

public:
	double X = 0;
	double Y = 0;
	double Degree = 0;
	double AllRangeDegree = 0;

	Position SelfPos;
	Position TargetPos;
	Position NextTargetPos;


	int MotorStepF = 0;
	int MotorStepT = 0;

	int TurnTableStep;

	//char MotorAction[3] = { 'S','S','S' };
	//double MotorValue[3] = { 0,0,0 };

	std::vector<Command> MotorCmd;
	std::vector<Position> RootPosSave;

	std::vector<cv::Point> RootClickPosSave;
	std::vector<cv::Point> DoorClickPosSave;
	std::vector<int> Root2ClickDir;

	int NowP2PCMD = -1;
	int NowPosMovFin = -1;

	int NextWeelDegree = 0;
	int NextTableDegree = 0;


public:
	void initial(int StepL, int StepR) {
		MotorStepL = StepL;
		MotorStepR = StepR;
		MotorStepF = 0;
		MotorStepT = 0;
		SelfPos.COORD.x = X;
		SelfPos.COORD.y = Y;
		SelfPos.Degree = Degree;
		MotorStepT = Degree * Weel_plus_per_deg;
		AllRangeDegree = Degree;

		Speed_L_Pre = 0;
		Speed_R_Pre = 0;

		Command tmpCMD;
		MotorCmd.assign(2, tmpCMD);


		printf("initial (%d,%d)\n", MotorStepL, MotorStepR);

	};
	void initial(int StepL, int StepR, double x, double y, double degree) {
		MotorStepL = StepL;
		MotorStepR = StepR;
		X = x;
		Y = y;
		Degree = degree;
		MotorStepF = 0;
		MotorStepT = 0;

		SelfPos.COORD.x = X;
		SelfPos.COORD.y = Y;
		SelfPos.Degree = Degree;
		MotorStepT = Degree * Weel_plus_per_deg;
		AllRangeDegree = Degree;

		Speed_L_Pre = 0;
		Speed_R_Pre = 0;

		printf("initial (%d,%d),(%.1f,%.1f,%.1f)\n", MotorStepL, MotorStepR, X, Y, Degree);
	};
	bool GetPos(Int32 StepL, Int32 StepR) {
		return GetPos(StepL, StepR, 1, 1);
	}
	bool GetPos(Int32 StepL, Int32 StepR, double time) {
		int step_tmpl = StepL - MotorStepL;
		int step_tmpr = StepR - MotorStepR;

		Int32 SpeedL_T = step_tmpl / time;
		Int32 SpeedR_T = step_tmpr / time;

		return GetPos(StepL, StepR, SpeedL_T, SpeedR_T);
	}
	bool GetPos(Int32 StepL, Int32 StepR, Int32 SpeedL, Int32 SpeedR) {
		steptmpl = StepL - MotorStepL;
		steptmpr = StepR - MotorStepR;

		int  ERROR_Code = 0;


		if (fabs(SpeedL) < 150) {
			SpeedL = 0.000001;
		}
		if (fabs(SpeedR) < 150) {
			SpeedR = 0.000001;
		}
		double Speed_L_F = SpeedL;
		double Speed_R_F = SpeedR;


		if (abs(steptmpl) < 50) {
			steptmpl = 0;
		}
		if (abs(steptmpr) < 50) {
			steptmpr = 0;
		}


		if (abs(steptmpl) > 500000) {
			ERROR_Code = 1;
		}
		else if (abs(steptmpr) > 500000) {
			ERROR_Code = 2;
		}
		else if (steptmpl != 0 && Speed_L_Pre == 0 && SpeedL == 0) {
			ERROR_Code = 3;
		}
		else if (steptmpr != 0 && Speed_R_Pre == 0 && SpeedR == 0) {
			ERROR_Code = 4;
		}

		if (ERROR_Code) {
			if (ERROR_Code_SAVE != ERROR_Code) {
				printf("GetPos ERROR CODE = %d \n", ERROR_Code);
				printf("ERROR DATA = (%d,%d)([%d,%d],[%d,%d]) \n"
					, steptmpl, steptmpr
					, Speed_L_Pre, SpeedL
					, Speed_R_Pre, SpeedR);
			}
			ERROR_Code_SAVE = ERROR_Code;
			return false;
		}
		ERROR_Code_SAVE = 0;

		MotorStepL = StepL;
		MotorStepR = StepR;




		double SpeedRate;

		if ((steptmpl || steptmpr) || (Speed_L_F || Speed_R_F)) {
			if (Speed_L_F == 0)
				Speed_L_F = 0.000001;
			if (Speed_R_F == 0)
				Speed_R_F = 0.000001;

			if (Speed_L_F > Speed_R_F)
				SpeedRate = ((fabs(Speed_L_F) - fabs(Speed_R_F)) / (double)Speed_L_F);
			else
				SpeedRate = ((fabs(Speed_L_F) - fabs(Speed_R_F)) / (double)Speed_R_F);

			//printf("L = %f , R = %f, SpeedRate = %f_%f,%f __ ", Speed_L_F, Speed_R_F,SpeedRate, abs(SpeedRate), fabs(SpeedRate));

			if (fabs(SpeedRate) < 0.05) {
				//printf("SpeedRate < 0.05\n");
				if (steptmpl > 0 && steptmpr < 0) {  //FWD
					changerate = Weel_plus_per_mm;
					X = X + (steptmpl *cos(Degree * M_PI / 180.0) / changerate);
					Y = Y + (steptmpl *sin(Degree * M_PI / 180.0) / changerate);
					MotorStepF += abs(steptmpl);
				}
				else if (steptmpl < 0 && steptmpr > 0) { //BACK
					changerate = -Weel_plus_per_mm;
					X = X - (steptmpl *cos(Degree * M_PI / 180.0) / changerate);
					Y = Y - (steptmpl *sin(Degree * M_PI / 180.0) / changerate);
					MotorStepF += steptmpl;
				}
				else if (steptmpl < 0 && steptmpr < 0) { //Left
					MotorStepT -= steptmpl;
					Degree = (MotorStepT / Weel_plus_per_deg);
					AllRangeDegree = Degree;
				}
				else if (steptmpl > 0 && steptmpr > 0) { //Right
					MotorStepT -= steptmpl;
					Degree = (MotorStepT / Weel_plus_per_deg);
					AllRangeDegree = Degree;
				}
			}
			else {

				//printf("SpeedRate > 0.05\n");

				if (steptmpl > 0 && steptmpr < 0) {  //FWD
					double r1, rc, L;
					double sitaR, sitaD;
					double move_L;

					if (abs(steptmpl) > abs(steptmpr)) {
						SpeedRate = fabs((double)Speed_L_F / (double)Speed_R_F);
						r1 = CAR_WELL_to_Weel_L / (SpeedRate - 1);
						L = fabs(steptmpl / Weel_plus_per_mm);
						sitaR = -L / (r1 + CAR_WELL_to_Weel_L);
					}
					else {
						SpeedRate = fabs((double)Speed_R_F / (double)Speed_L_F);
						r1 = CAR_WELL_to_Weel_L / (SpeedRate - 1);
						L = fabs(steptmpr / Weel_plus_per_mm);
						sitaR = L / (r1 + CAR_WELL_to_Weel_L);
					}
					sitaD = 0.5 * sitaR * 180 / M_PI; // RAD to DEG

					MotorStepT += sitaD * Weel_plus_per_deg;
					Degree = (MotorStepT / Weel_plus_per_deg);
					SelfPos.Degree += sitaD;
					AllRangeDegree = Degree;

					rc = r1 + (0.5*CAR_WELL_to_Weel_L);
					move_L = 2 * rc* sin(fabs(sitaR)*0.5);
					X = X + (move_L *cos(Degree*M_PI / 180.0));
					Y = Y + (move_L *sin(Degree*M_PI / 180.0));
					MotorStepF += move_L / Weel_plus_per_mm;

					MotorStepT += sitaD * Weel_plus_per_deg;
					Degree = (MotorStepT / Weel_plus_per_deg);
					SelfPos.Degree -= sitaD;
					AllRangeDegree = Degree;
				}
				else if (steptmpl < 0 && steptmpr > 0) { //BACK
					double r1, rc, L;
					double sitaR, sitaD;
					double move_L;

					if (abs(steptmpl) > abs(steptmpr)) {
						SpeedRate = fabs((double)Speed_L_F / (double)Speed_R_F);
						r1 = CAR_WELL_to_Weel_L / (SpeedRate - 1);
						L = fabs(steptmpl / Weel_plus_per_mm);
						sitaR = L / (r1 + CAR_WELL_to_Weel_L);
					}
					else {
						SpeedRate = fabs((double)Speed_R_F / (double)Speed_L_F);
						r1 = CAR_WELL_to_Weel_L / (SpeedRate - 1);
						L = fabs(steptmpr / Weel_plus_per_mm);
						sitaR = -L / (r1 + CAR_WELL_to_Weel_L);
					}
					sitaD = 0.5 * sitaR * 180 / M_PI; // RAD to DEG

					MotorStepT += sitaD * Weel_plus_per_deg;
					Degree = (MotorStepT / Weel_plus_per_deg);
					SelfPos.Degree += sitaD;
					AllRangeDegree = Degree;

					rc = r1 + (0.5*CAR_WELL_to_Weel_L);
					move_L = 2 * rc* sin(fabs(sitaR)*0.5);
					X = X - (move_L *cos(Degree*M_PI / 180.0));
					Y = Y - (move_L *sin(Degree*M_PI / 180.0));
					MotorStepF += move_L / Weel_plus_per_mm;

					MotorStepT += sitaD * Weel_plus_per_deg;
					Degree = (MotorStepT / Weel_plus_per_deg);
					SelfPos.Degree += sitaD;
					AllRangeDegree = Degree;
				}
				else if (steptmpl < 0 && steptmpr < 0) { //Left
					MotorStepT -= steptmpl;
					Degree = (MotorStepT / Weel_plus_per_deg);
					AllRangeDegree = Degree;
				}
				else if (steptmpl > 0 && steptmpr > 0) { //Right
					MotorStepT -= steptmpl;
					Degree = (MotorStepT / Weel_plus_per_deg);
					AllRangeDegree = Degree;
				}
			}
		}

		SelfPos.COORD.x = X;
		SelfPos.COORD.y = Y;
		while (Degree < 0)
			Degree += 360.0;
		while (Degree > 360)
			Degree -= 360.0;
		SelfPos.Degree = Degree;

		Speed_L_Pre = Speed_L_F;
		Speed_R_Pre = Speed_R_F;

		return true;

	}

	void SetTarget(double x, double y, double degree) {
		TargetPos.COORD.x = x;
		TargetPos.COORD.y = y;
		TargetPos.Degree = degree;
	}
	void NextTargetSet(double x, double y, double degree) {
		NextTargetPos.COORD.x = x;
		NextTargetPos.COORD.y = y;
		NextTargetPos.Degree = degree;
	}
	void NextTargetSet(Position Pos) {
		NextTargetPos.COORD.x = Pos.COORD.x;
		NextTargetPos.COORD.y = Pos.COORD.y;
		NextTargetPos.Degree = Pos.Degree;
	}
	void AdjustSelfPosion(double x, double y, double degree) {

		MotorStepT += degree * Weel_plus_per_deg;
		Degree = (MotorStepT / Weel_plus_per_deg);
		AllRangeDegree = Degree;
		SelfPos.Degree += degree;
		AdjustSelfPosion(x, y);
	}
	void AdjustSelfPosion(double x, double y) {
		X += x;
		Y += y;
		SelfPos.COORD.x = X;
		SelfPos.COORD.y = Y;
	}

	double Point2Point(Position Self, Position Target, Command *MoCmd, int step) {
		double deltaX = Target.COORD.x - Self.COORD.x;
		double deltaY = Target.COORD.y - Self.COORD.y;
		double deltaDegree = Target.Degree - Self.Degree;

		double nowD = Self.Degree;

		//int table = TurnTableStep;
		//int tabletmp, tableNow;


		double MoveDir = atan(deltaY / deltaX)*(180.0 / M_PI);
		double distance = pow(pow(deltaX, 2) + pow(deltaY, 2), 0.5);

		if (deltaX < 0)
			MoveDir += 180;
		while (MoveDir < 0)
			MoveDir += 360;
		while (MoveDir > 360)
			MoveDir -= 360;


		double tmpD2;

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

		tmpD2 = savetmd2_work;

		if (tmpD2 > P2P_THRESHOLD) {
			MoCmd[0].Action = 'L';
			nowD = nowD + tmpD2;
		}
		else if (tmpD2 < -P2P_THRESHOLD) {
			MoCmd[0].Action = 'R';
			nowD = nowD + tmpD2;
		}
		else {
			MoCmd[0].Action = 'S';
		}
		MoCmd[0].Value = fabs(tmpD2);


		///
		if (step > 1) {
			if (distance > 10) {
				MoCmd[1].Action = 'F';
			}
			else {
				MoCmd[1].Action = 'S';
				if (fabs(tmpD2) > P2P_THRESHOLD && MoCmd[0].Action != 'S') {
					MoCmd[0].Action = 'S';
					nowD = nowD - tmpD2;
				}

			}
			MoCmd[1].Value = distance;
		}
		else {
			MoCmd[1].Action = 'S';
			MoCmd[1].Value = 0;
		}
		///
		return nowD;
	}
	void Point2Point_OnePoint(void) {
		MotorCmd.clear();
		Command tmpCMD;
		MotorCmd.assign(2, tmpCMD);
		Point2Point(SelfPos, TargetPos, &MotorCmd[0], 2);

	}

	void Point_Move_Action_Clear(void) {
		if (!MotorCmd.empty())
			MotorCmd.clear();
		NowP2PCMD = -1;
	}
	void NextPosWeelAndTable(double Direction) {
		double savetmd2_1, savetmd2_2, savetmd2_work;

		double deltaX = NextTargetPos.COORD.x - SelfPos.COORD.x;
		double deltaY = NextTargetPos.COORD.y - SelfPos.COORD.y;
		double MoveDir = atan(deltaY / deltaX)*(180.0 / M_PI);
		if (deltaX < 0)
			MoveDir += 180;

		while (MoveDir < 0)
			MoveDir += 360;
		while (MoveDir > 360)
			MoveDir -= 360;

		savetmd2_1 = MoveDir - SelfPos.Degree;
		if (savetmd2_1 > 0)
			savetmd2_2 = savetmd2_1 - 360;
		else if (savetmd2_1 < 0)
			savetmd2_2 = 360 + savetmd2_1;
		else
			savetmd2_2 = 0;

		if (fabs(savetmd2_1) > fabs(savetmd2_2))
			savetmd2_work = savetmd2_2;
		else
			savetmd2_work = savetmd2_1;
		NextWeelDegree = savetmd2_work;

		int table = TurnTableStep;
		double tableDeg = table * 360 / 150000;
		double tableDir = MoveDir + tableDeg;
		while (tableDir < 0)
			tableDir += 360;
		while (tableDir > 360)
			tableDir -= 360;

		savetmd2_1 = Direction - tableDir;
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

		int tabletmp = savetmd2_work * 150000 / 360;
		if (abs(table + tabletmp) >= 150000) {
			savetmd2_work = savetmd2_2;
		}

		NextTableDegree = savetmd2_work;

	}

	double CircleDelta(double Target, double Self) {
		double Delta1 = Target - Self;
		double Delta2;
		if (Delta1 > 0)
			Delta2 = Delta1 - 360;
		else if (Delta1 < 0)
			Delta2 = 360 + Delta1;
		else
			Delta2 = 0;

		if (fabs(Delta1) < fabs(Delta2))
			return Delta1;
		else
			return Delta2;

	};
	double TableDirectionNow(void) {
		double selfdeg = SelfPos.Degree;
		double tableDeg = TurnTableStep * 360 / 150000;
		double TableDirNow = selfdeg + tableDeg;

		while (TableDirNow < 0)
			TableDirNow += 360;
		while (TableDirNow > 360)
			TableDirNow -= 360;
		return TableDirNow;
	}
	int TableRotValue(double Target) {
		double savetmd2_1, savetmd2_2, savetmd2_work;
		double selfdeg = SelfPos.Degree;

		int table = TurnTableStep;
		double tableDeg = table * 360 / 150000;
		double tableDir = selfdeg + tableDeg;
		while (tableDir < 0)
			tableDir += 360;
		while (tableDir > 360)
			tableDir -= 360;

		savetmd2_1 = Target - tableDir;
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

		int tabletmp = savetmd2_work * 150000 / 360;
		if (abs(table + tabletmp) >= 150000) {
			savetmd2_work = savetmd2_2;
		}

		NextTableDegree = savetmd2_work;
		return savetmd2_work;
	}

	int SetRootPos(double x, double y, double d = -1) {
		Position pos;
		pos.COORD.x = x;
		pos.COORD.y = y;
		pos.Degree = d;
		RootPosSave.push_back(pos);
		return RootPosSave.size();
	}
	int SetRootClickPos(double x, double y, double d = -1) {
		cv::Point pos = cv::Point(x, y);
		RootClickPosSave.push_back(pos);
		return RootClickPosSave.size();
	}
	int SetDoorClickPos(double x, double y, double d = -1) {
		cv::Point pos = cv::Point(x, y);
		DoorClickPosSave.push_back(pos);
		return DoorClickPosSave.size();
	}
	void RootClear() {
		if (!RootPosSave.empty())
			RootPosSave.clear();
		if (!MotorCmd.empty())
			MotorCmd.clear();
	}

	void RootClickClear() {
		if (!RootClickPosSave.empty())
			RootClickPosSave.clear();
	}
	void DoorClickClear() {
		if (!DoorClickPosSave.empty())
			DoorClickPosSave.clear();
	}

	bool Pos_2_RootCalculate(int step = 0) {
		if (RootPosSave.size() == 0)
			return false;

		Position self;
		Position target;
		Command tmpCMD;
		double tmpDegree;
		int start = 0;

		self = this->SelfPos;

		MotorCmd.clear();
		if (step == 0)
			MotorCmd.assign(RootPosSave.size() * 2, tmpCMD);
		else
			MotorCmd.assign(step * 2, tmpCMD);
		if (step == 0) {
			//全部路徑計算
			for (int i = 0; i < RootPosSave.size(); i++) {
				target = RootPosSave.at(i);
				tmpDegree = Point2Point(self, target, &MotorCmd[i * 2], 2);
				self = target;
				self.Degree = tmpDegree;
			}
		}
		else {
			//從未PASS重新計算
			for (int i = 0; i < RootPosSave.size(); i++) {
				if (!RootPosSave.at(i).passed) {
					start = i;
					break;
				}
			}
			for (int i = 0; (i < step) && (start + i < RootPosSave.size()); i++) {
				target = RootPosSave.at(start + i);
				tmpDegree = Point2Point(self, target, &MotorCmd[i * 2], 2);
				self = target;
				self.Degree = tmpDegree;
			}
		}
		for (int i = 0; i < MotorCmd.size(); i++) {
			printf("act[%d]_[%c][%f]\n", i, MotorCmd.at(i).Action, MotorCmd.at(i).Value);
		}
	}

	int Distance_2_Motor_Step(char type, double Distance) {
		if (type == 'F' || type == 'B') {
			Distance = Distance * Weel_plus_per_mm;

		}
		else if (type == 'L' || type == 'R') {
			Distance = Distance * Weel_plus_per_deg;
		}
		return Distance;
	}


};
//}

#endif
