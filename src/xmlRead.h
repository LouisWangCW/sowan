#pragma once
#include <tinyxml.h>
//#include "tinystr.h"
#include <string>

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <vector>
#include "def.hpp"

//using namespace std;

class SOWAN_PARAMETER {
private:
	std::string Home_Root_String = HOME_ROOT;
	std::string defaultVoicePath = Home_Root_String+"/Voice/DefaultVoice.wav";
public:

	std::vector<int> Point_Route;
	std::vector<int> Point_Charge;
	std::vector<int> Point_Room;
	std::vector<std::string> RoomVoice;

	std::vector<int> RoomVoiceTime;

	std::string BlockImgPath;
	std::string PatrolVoicePath = defaultVoicePath;
	std::string VehicleStopVoicePath = defaultVoicePath;

	int Route_Standby_Time = 1;
	

	double FrontSafeDistance = 0.4;
	double BackSafeDistance = 0.4;
	double SideSafeDistance = 0.25;
	double ArrivalRange = 0.1;
	double MaxCircleSpd = 0.33;
	double MaxLineSpd = 0.2;
	double MaxTurnSpd = 0.2;
	double MinCircleNeedDeg = 5;
	double MinTurnNeedDeg = 30;
	double normalMovDegOffset = 3;

	double PatrolOverDeg = 120;
	double PatrolOverDeg_FinDeg = 45;
	
	double BackMovEscapeDistance = 0.1;
	double DoorOpenWaitTime = 5.5;

	double DoorOpenDistance = 1.5;
	double DoorOpenWidth = 0.4;

	int PatrolCycleCount = 1;
	int PatrolCycleWatiTime = 0;
	int WorkLoopTime = -1;
	int WorkLoopSleepTime = 0;
	int ChargeWaitTime = 0;
	int PatrolVoiceTime = 5;
	int HitObjectRunawayType = 0;
	int normalMoveType = 0;
	int VehicleStopVoiceTime =5;

	

	double acc = 1500;
	double spd = 400;
	double turnRate = 0.2;

	bool HitObjectStop = true;
	bool Debug_SW = false;
	bool PatrolVoiceSwitch = false;
	bool ChargePlatform = false;
	bool AutoMotorOn = false;
	bool Debug_ShowReadError = false;
	bool LowSensorInstall = false;

	


	SOWAN_PARAMETER(std::string file) {
		LoadXml(file);
		//LoadXml_Config();
	}
	~SOWAN_PARAMETER() {
	
	}

	void LoadXml_Config()
	{
		std::string file = Home_Root_String +"/map/sowan.xml";
		TiXmlDocument xmlDoc(file.c_str());

		xmlDoc.LoadFile();

		if (xmlDoc.ErrorId() > 0)
			return;

		TiXmlElement* pRootElement = xmlDoc.RootElement();

		if (!pRootElement)
			return;

		TiXmlElement* pSOWAN = NULL;
		TiXmlElement* pNode = NULL;

		std::string nodeName;

		

	}

	void LoadXml(std::string file)
	{
		std::string str_P;
		std::string tmp;

		TiXmlDocument xmlDoc(file.c_str());

		xmlDoc.LoadFile();

		if (xmlDoc.ErrorId() > 0)
			return;

		TiXmlElement* pRootElement = xmlDoc.RootElement();

		if (!pRootElement)
			return;

		TiXmlElement* pSOWAN = NULL;
		TiXmlElement* pNode = NULL;

		std::string nodeName;


		pSOWAN = pRootElement->FirstChildElement("Patrol_Point");
		if (pSOWAN)
		{
			nodeName = "SET_";
			std::string num, numdir;
			std::string tmp;
			int str_dir[3];

			for (int i = 1; i < 99; i++) {

				num = nodeName + std::to_string(i);
				pNode = pSOWAN->FirstChildElement(num.c_str());
				if (pNode) {

					numdir = pNode->GetText();
					str_dir[0] = numdir.find('(');
					str_dir[1] = numdir.find(',');
					str_dir[2] = numdir.find(')');
					tmp = tmp.assign(numdir, str_dir[0] + 1, str_dir[1] - str_dir[0] - 1);
					Point_Route.push_back(atoi(tmp.c_str()));
					tmp = tmp.assign(numdir, str_dir[1] + 1, str_dir[2] - str_dir[1] - 1);
					Point_Route.push_back(atoi(tmp.c_str()));

				}
				else {
					break;
				}
			}
		}

		pSOWAN = pRootElement->FirstChildElement("Room_Set");
		if (pSOWAN) {

			nodeName = "ROOM_SET_";
			TiXmlElement* pRoom = NULL;
			std::string ROOM_SET, ROOM_SET_P;
			std::string VoiceString;

			int str_dir[3];
			std::vector<int> vectmp;
			vectmp.assign(8, 0);

			for (int i = 1; i < 99; i++) {
				ROOM_SET = nodeName + std::to_string(i);
				pRoom = pSOWAN->FirstChildElement(ROOM_SET.c_str());
				if (pRoom) {
					for (int j = 0; j < 4; j++) {
						ROOM_SET_P = "SET_" + std::to_string(1+j);
						pNode = pRoom->FirstChildElement(ROOM_SET_P.c_str());
						if (pNode) {
							str_P = pNode->GetText();
							str_dir[0] = str_P.find('(');
							str_dir[1] = str_P.find(',');
							str_dir[2] = str_P.find(')');

							tmp = tmp.assign(str_P, str_dir[0] + 1, str_dir[1] - str_dir[0] - 1);
							Point_Room.push_back(atoi(tmp.c_str()));
							vectmp.at(2 * j) = atoi(tmp.c_str());

							tmp = tmp.assign(str_P, str_dir[1] + 1, str_dir[2] - str_dir[1] - 1);
							Point_Room.push_back(atoi(tmp.c_str()));
							vectmp.at(2 * j + 1) = atoi(tmp.c_str());

						}
						else {
							Point_Room.push_back(0);
							Point_Room.push_back(0);
						}
					}

					pNode = pRoom->FirstChildElement("RoomVoice");
					if (pNode) {
						str_P = pNode->GetText();
						RoomVoice.push_back(str_P);
					}
					else {
						RoomVoice.push_back(defaultVoicePath);
					}

					pNode = pRoom->FirstChildElement("RoomVoiceTime");
					if (pNode) {
						RoomVoiceTime.push_back(atoi(pNode->GetText()));
					}
					else {
						RoomVoiceTime.push_back(5);
					}
				}
				else {
					break;
				}
			}
		}

		pSOWAN = pRootElement->FirstChildElement("Charge_Point");
		if (pSOWAN)
		{
			nodeName = "SET_";
			std::string num, numdir;
			std::string tmp;
			int str_dir[3];

			for (int i = 1; i < 3; i++) {

				num = nodeName + std::to_string(i);
				pNode = pSOWAN->FirstChildElement(num.c_str());
				if (pNode) {

					numdir = pNode->GetText();
					str_dir[0] = numdir.find('(');
					str_dir[1] = numdir.find(',');
					str_dir[2] = numdir.find(')');
					tmp = tmp.assign(numdir, str_dir[0] + 1, str_dir[1] - str_dir[0] - 1);
					Point_Charge.push_back(atoi(tmp.c_str()));
					tmp = tmp.assign(numdir, str_dir[1] + 1, str_dir[2] - str_dir[1] - 1);
					Point_Charge.push_back(atoi(tmp.c_str()));

				}
				else {
					break;
				}
			}
		}

		
		pSOWAN = pRootElement->FirstChildElement("BlockImgPath");
		if (pSOWAN) {
			BlockImgPath = pSOWAN->GetText();
		}

		pSOWAN = pRootElement->FirstChildElement("PatrolCycleCount");
		if (pSOWAN) {
			PatrolCycleCount = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("PatrolCycleWatiTime");
		if (pSOWAN) {
			PatrolCycleWatiTime = atoi(pSOWAN->GetText());
		}

		pSOWAN = pRootElement->FirstChildElement("WorkLoopTime");
		if (pSOWAN) {
			WorkLoopTime = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("WorkLoopSleepTime");
		if (pSOWAN) {
			WorkLoopSleepTime = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("ChargeWaitTime");
		if (pSOWAN) {
			ChargeWaitTime = atoi(pSOWAN->GetText());
		}

		/////////
		pSOWAN = pRootElement->FirstChildElement("FrontSafeDistance");
		if (pSOWAN) {
			FrontSafeDistance = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("BackSafeDistance");
		if (pSOWAN) {
			BackSafeDistance = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("SideSafeDistance");
		if (pSOWAN) {
			SideSafeDistance = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("ArrivalRange");
		if (pSOWAN) {
			ArrivalRange = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("MaxCircleSpd");
		if (pSOWAN) {
			MaxCircleSpd = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("MaxLineSpd");
		if (pSOWAN) {
			MaxLineSpd = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("MaxTurnSpd");
		if (pSOWAN) {
			MaxTurnSpd = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("MinCircleNeedDeg");
		if (pSOWAN) {
			MinCircleNeedDeg = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("MinTurnNeedDeg");
		if (pSOWAN) {
			MinTurnNeedDeg = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("PatrolOverDeg");
		if (pSOWAN) {
			PatrolOverDeg = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("PatrolOverDeg_FinDeg");
		if (pSOWAN) {
			PatrolOverDeg_FinDeg = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("normalMovDegOffset");
		if (pSOWAN) {
			normalMovDegOffset = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("acc");
		if (pSOWAN) {
			acc = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("spd");
		if (pSOWAN) {
			spd = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("turnRate");
		if (pSOWAN) {
			turnRate = strtod(pSOWAN->GetText(), NULL);
		}

		pSOWAN = pRootElement->FirstChildElement("HitObjectStop");
		if (pSOWAN) {
			int tmpHitObjectStop = atoi(pSOWAN->GetText());
			if (tmpHitObjectStop != 0) {
				HitObjectStop = false;
			}
		}

		pSOWAN = pRootElement->FirstChildElement("Debug_SW");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				Debug_SW = false;
			}
			else {
				Debug_SW = true;
			}
		}

		pSOWAN = pRootElement->FirstChildElement("DoorOpenDistance");
		if (pSOWAN) {
			DoorOpenDistance = strtod(pSOWAN->GetText(), NULL);
		}
		pSOWAN = pRootElement->FirstChildElement("DoorOpenWidth");
		if (pSOWAN) {
			DoorOpenWidth = strtod(pSOWAN->GetText(), NULL);
		}
		
		pSOWAN = pRootElement->FirstChildElement("BackMovEscapeDistance");
		if (pSOWAN) {
			BackMovEscapeDistance = strtod(pSOWAN->GetText(), NULL);
		}
		
		pSOWAN = pRootElement->FirstChildElement("PatrolVoiceSwitch");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				PatrolVoiceSwitch = false;
			}
			else {
				PatrolVoiceSwitch = true;
			}
		}
		pSOWAN = pRootElement->FirstChildElement("PatrolVoiceTime");
		if (pSOWAN) {
			PatrolVoiceTime = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("normalMoveType");
		if (pSOWAN) {
			normalMoveType = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("PatrolVoicePath");
		if (pSOWAN) {
			str_P = pSOWAN->GetText();
			PatrolVoicePath = str_P;
		}
		pSOWAN = pRootElement->FirstChildElement("VehicleStopVoiceTime");
		if (pSOWAN) {
			VehicleStopVoiceTime = atoi(pSOWAN->GetText());
		}
		pSOWAN = pRootElement->FirstChildElement("VehicleStopVoicePath");
		if (pSOWAN) {
			str_P = pSOWAN->GetText();
			VehicleStopVoicePath = str_P;
		}
		pSOWAN = pRootElement->FirstChildElement("ChargePlatform");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				ChargePlatform = false;
			}
			else {
				ChargePlatform = true;
			}
		}
		pSOWAN = pRootElement->FirstChildElement("AutoMotorOn");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				AutoMotorOn = false;
			}
			else {
				AutoMotorOn = true;
			}
		}
		pSOWAN = pRootElement->FirstChildElement("Debug_ShowReadError");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				Debug_ShowReadError = false;
			}
			else {
				Debug_ShowReadError = true;
			}
		}
		pSOWAN = pRootElement->FirstChildElement("LowSensorInstall");
		if (pSOWAN) {
			int tmpDebug_SW = atoi(pSOWAN->GetText());
			if (tmpDebug_SW == 0) {
				LowSensorInstall = false;
			}
			else {
				LowSensorInstall = true;
			}
		}

		pSOWAN = pRootElement->FirstChildElement("HitObjectRunawayType");
		if (pSOWAN) {
			HitObjectRunawayType = atoi(pSOWAN->GetText());
		}
		
		
		/////////

		//xmlDoc.Clear();
		printf("Load File Finish\n");
	}


};
