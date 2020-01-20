#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

#ifndef __ROUTE_SEARCH___
#define __ROUTE_SEARCH__


#define MAP_SPACE 200
#define MAP_OBJECT 63
#define MAP_PASSING 128
#define MAP_PASSED 64

#define MonitorCounter 5000

#define BlockMinValue 1

//using namespace cv;

class MapRoute {

public:

	float MeterPerPix = 0.05;  // meter/pix
	float RobotSizeReal = 0.48; //  480mm
	std::vector<cv::Point> RouteOri;
	std::vector<cv::Point> RouteNew;

	cv::Rect MapRect;

	cv::Mat *Map_BlockValue;
	cv::Mat Monitor;

private:
	cv::Mat LowPriority_Original;	// Low 0xff Space&Obj 0x0
	cv::Mat Map_Original;			// Obj 0x00 Space 0xff
	cv::Mat Map_Original_rev;		// Obj 0xff Space 0x00

	cv::Mat Monitor_Ori;
	int RobotSizePix = RobotSizeReal / MeterPerPix;
	cv::Mat RouteFlow;
	std::vector<cv::Point> routerTmp;

	cv::Mat *Map_BlockOri;
	cv::Mat Map_Block_Calculated;

	std::vector<uchar> Map_BlockRoute;

	std::vector<cv::Point> BlockCenter;

	cv::Mat Map_Block_Tmp;

	bool MapInitFinished = false;
	bool BlockInitFinished = false;
	bool MapRectFinished = false;
	bool BlockRectFinished = false;


public:
	MapRoute() {

	}
	void MapInit(cv::Mat *ORI) {
		cv::Mat Map_tmp;
		cv::Mat kernal = cv::Mat(3, 3, CV_8UC1);

		if (MapInitFinished == false) {
			kernal = cv::Scalar(255);
			if (ORI->channels() == 1) {
				ORI->copyTo(Map_tmp);
				cv::cvtColor(*ORI, Monitor_Ori, CV_GRAY2BGR);
			}
			else {
				ORI->copyTo(Monitor_Ori);
				cv::cvtColor(*ORI, Map_tmp, CV_BGR2GRAY);
			}

			Monitor_Ori.copyTo(Monitor);

			cv::threshold(Map_tmp, Map_tmp, 200, 255, cv::THRESH_BINARY);
			erode(Map_tmp, Map_Original, kernal, cv::Point(-1, -1), 1);
			dilate(Map_Original, Map_tmp, kernal, cv::Point(-1, -1), 1);

			kernal = cv::Mat(1.0 * RobotSizePix + 1, 1.0* RobotSizePix + 1, CV_8UC1);
			kernal = cv::Scalar(255);
			erode(Map_tmp, Map_Original, kernal, cv::Point(-1, -1), 1);

			kernal = cv::Mat(1.0 * RobotSizePix + 1, 1.0 * RobotSizePix + 1, CV_8UC1);
			kernal = cv::Scalar(255);
			erode(Map_Original, LowPriority_Original, kernal, cv::Point(-1, -1), 1);
			cv::bitwise_xor(LowPriority_Original, Map_Original, LowPriority_Original);

			Map_Original_rev = ~Map_Original;

			RouteFlow = cv::Mat::zeros(Map_Original.size(), CV_16U);
		}

		MapInitFinished = true;
		
	}

	void Refresh() {
		Monitor_Ori.copyTo(Monitor);	
	}
	void BlockMapInit(cv::Mat *BlockMap) {
		BlockMap->copyTo(Map_Block_Tmp);
		BlockInitFinished = true;
		printf("BlockMapInit Finished\n");
	}
	void GetMapRect(cv::Rect rect) {
		MapRect = rect;
		MapRectFinished = true;
	}
	bool AllMapCheck() {
		if (MapInitFinished && MapRectFinished && BlockInitFinished && !BlockRectFinished) {
			
			cv::Mat tmp;

			Map_Block_Tmp(MapRect).copyTo(tmp);
			//cv::imshow("Map_Block_Tmp", tmp);
			//cv::waitKey(1);
			printf("Map_Block_Tmp Finished\n");
			BlockCutAndSetDir(&tmp);
			printf("BlockCutAndSetDir Finished\n");

			Map_BlockValue = &Map_Block_Calculated;
			GetBlockCenter();
			printf("GetBlockCenter Finished\n");

			BlockRectFinished = true;
			return true;
		}
		else if (MapInitFinished && MapRectFinished && BlockInitFinished && BlockRectFinished) {
			return true;
		}
		return false;
	}



	void ReturnMapCanMove(cv::Mat *map) {
		cv::Mat Map_tmp;
		Monitor.copyTo(Map_tmp);

		for (int y = 0; y < Map_Original.rows ; y++) {
			for (int x = 0; x < Map_Original.cols ; x++) {
				if (Map_Original.at<uchar>(y, x) != 255) {
					Map_tmp.data[y * Map_tmp.step + x * Map_tmp.elemSize()] *= 0.2;
					Map_tmp.data[y * Map_tmp.step + x * Map_tmp.elemSize()+1] *= 0.2;
					Map_tmp.data[y * Map_tmp.step + x * Map_tmp.elemSize()+2] *= 0.2;
				}

			}
		}
		Map_tmp.copyTo(*map);
	}

	void BlockCutAndSetDir(cv::Mat *Map_BlockOri) {

		
		int value_dir;
		double min, max;
		cv::Point center;
		cv::Moments mu;

		cv::Mat BlockMask;

		cv::Mat Map_bin_Block_Ori;

		cv::inRange(*Map_BlockOri, cv::Scalar(0,0,255), cv::Scalar(0,0,255), Map_bin_Block_Ori);

		int get_count = BlockMinValue;
		int tmpColor;
		bool breakflag = false;
		

		for (int y = 0; y < Map_bin_Block_Ori.rows && !breakflag; y++) {
			for (int x = 0; x < Map_bin_Block_Ori.cols && !breakflag; x++) {
				tmpColor = (0x00ff & (int)Map_bin_Block_Ori.at<uchar>(y, x));

				//tmpColor = Map_bin_Block_Ori.data[y * Map_bin_Block_Ori.cols + x];
				if (tmpColor == 0x00ff) {
					cv::floodFill(Map_bin_Block_Ori, cv::Point(x,y),cv::Scalar(get_count), (cv::Rect*)0, cv::Scalar(),0);
					get_count++;
				}
			}
		}
		Map_bin_Block_Ori.copyTo(Map_Block_Calculated);

	}

	void GetBlockCenter() {
		cv::Mat Map_tmp;
		int value_dir;
		double min, max;
		cv::Point center;
		cv::Moments mu;

		for (int i = BlockMinValue; i < 255; i++) {
			//cv::threshold(*MapTestFuc, Map_tmp, i, 255, cv::THRESH_BINARY);
			cv::inRange(*Map_BlockValue, cv::Scalar(i), cv::Scalar(i), Map_tmp);
			cv::minMaxLoc(Map_tmp, &min, &max);
			if (max < BlockMinValue) {
				break;
			}
			else {
				mu = cv::moments(Map_tmp);
				center = cv::Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
				circle(Map_tmp, center, 8, cv::Scalar(140), 3, 8, 0);
				BlockCenter.push_back(center);
			}
		}

		
	}

	bool FindRoute(cv::Mat *Map, int *tx, int *ty) {
		int y = *ty;
		int x = *tx;
		int min_val = 10;
		int min_dir = 0;

		bool test = false;


		int dif[9];
		for (int i = 0; i < 9; i++) {
			dif[i] = Map->at<short>(y, x) - Map->at<short>(y + (i / 3) - 1, x + (i % 3) - 1);
			if (dif[i] < min_val && dif[i] > 0 && dif[i] < 10) {
				min_dir = i;
				min_val = dif[i];
			}
			if (i != 8 && dif[i] == 0) {
				test = true;
			}
		}
		//012
		//345
		//678

		if (min_val < 10) {
			if (dif[7] == min_val)
				y++;
			else if (dif[1] == min_val)
				y--;
			else if (dif[5] == min_val)
				x++;
			else if (dif[3] == min_val)
				x--;
			else if (dif[8] == min_val) {
				y++;
				x++;
			}
			else if (dif[0] == min_val) {
				y--;
				x--;
			}
			else if (dif[2] == min_val) {
				x++;
				y--;
			}
			else if (dif[6] == min_val) {
				x--;
				y++;
			}
		}
		else {  // For DEBUG

			if (test) {
				for (int i = 0; i < 9; i++) {
					dif[i] += 1;
				}
				min_val = 1;
				if (dif[7] == min_val)
					y++;
				else if (dif[1] == min_val)
					y--;
				else if (dif[5] == min_val)
					x++;
				else if (dif[3] == min_val)
					x--;
				else if (dif[8] == min_val) {
					y++;
					x++;
				}
				else if (dif[0] == min_val) {
					y--;
					x--;
				}
				else if (dif[2] == min_val) {
					x++;
					y--;
				}
				else if (dif[6] == min_val) {
					x--;
					y++;
				}
				printf(" special (%d,%d_%d)\n", x, y, Map->at<short>(y, x));
				for (int q = 0; q < 9; q++)
					printf("(%d,%d_%d=%d) ",
						x - 1 + (q % 3),
						y - 1 + (q / 3),
						Map->at<short>(y - 1 + (q / 3), x - 1 + (q % 3)),
						dif[q]
					);
				printf(" \n");
				*ty = y;
				*tx = x;
				return false;
			}
			else {
				printf(" stop (%d,%d_%d)\n", x, y, Map->at<short>(y, x));
				for (int q = 0; q < 9; q++)
					printf("(%d,%d_%d=%d) ",
						x - 1 + (q % 3),
						y - 1 + (q / 3),
						Map->at<short>(y - 1 + (q / 3), x - 1 + (q % 3)),
						dif[q]
					);
				printf(" \n");
				return false;
			}
		}
		*ty = y;
		*tx = x;
		return true;

	}

	bool FindArea(cv::Mat *Map, cv::Mat *LowPriority, int x, int y) {
		int tmp = 0;
		int priority = 0;
		int tx;
		int ty;
		int ptr;
		bool Area_End = true;
		//甤
		for (int q = 0; q < 9; q++) {

			tx = (x + (q % 3) - 1);
			ty = (y + (q / 3) - 1);
			ptr = ty * Map->cols + tx;
			tmp = Map->data[ptr];
			priority = LowPriority->data[ptr];

			if (q == 4) {
				continue;
			}
			else if (priority > 0) {
				LowPriority->data[ptr] = 128;
				Area_End = false;
			}
			else if (tmp > MAP_SPACE) {
				if (q & 1) {
					Map->data[ty * Map->cols + tx] = MAP_PASSING;
				}
				else if (q == 0) {
					if (Map->data[(ty)* Map->cols + (tx + 1)] > MAP_OBJECT && Map->data[(ty + 1) * Map->cols + (tx)] > MAP_OBJECT)
						Map->data[ty * Map->cols + tx] = MAP_PASSING;
				}
				else if (q == 2) {
					if (Map->data[(ty)* Map->cols + (tx - 1)] > MAP_OBJECT && Map->data[(ty + 1) * Map->cols + (tx)] > MAP_OBJECT)
						Map->data[ty * Map->cols + tx] = MAP_PASSING;
				}
				else if (q == 6) {
					if (Map->data[(ty)* Map->cols + (tx + 1)] > 63 && Map->data[(ty - 1) * Map->cols + (tx)] > MAP_OBJECT)
						Map->data[ty * Map->cols + tx] = MAP_PASSING;
				}
				else if (q == 8) {
					if (Map->data[(ty)* Map->cols + (tx - 1)] > MAP_OBJECT && Map->data[(ty - 1) * Map->cols + (tx)] > MAP_OBJECT)
						Map->data[ty * Map->cols + tx] = MAP_PASSING;
				}
			}
		}
		return Area_End;
	}

	bool AreaSearch(cv::Mat *Map_Now, cv::Mat *grayRoute, cv::Mat *Monitor, cv::Point StartP, cv::Point EndP) {

		bool keepwork = true;
		bool P_hit = false;
		bool AreaEnd = false;
		cv::Mat Map_Pre;
		cv::Mat  LowPriority_Pre;
		cv::Mat   LowPriority;
		int x = 0, y = 0;

		int RouteCount = 1;
		int ShowCount = 0;

		LowPriority_Original.copyTo(LowPriority);
		LowPriority.copyTo(LowPriority_Pre);

		Map_Now->at<uchar>(StartP.y, StartP.x) = MAP_PASSING;
		Map_Now->copyTo(Map_Pre);


		while (keepwork) {
			keepwork = false;

			for (y = 1; y < Map_Now->rows - 1 && !P_hit; y++) {
				for (x = 1; x < Map_Now->cols - 1 && !P_hit; x++) {
					if (Map_Pre.data[y * Map_Pre.cols + x] == MAP_PASSING) {

						if (FindArea(Map_Now, &LowPriority, x, y)) {
							Map_Now->data[y * Map_Now->cols + x] = MAP_PASSED;
						}

						if (grayRoute->at<short>(y, x) == 0) {
							grayRoute->at<short>(y, x) = RouteCount;
						}

						Monitor->data[y * Monitor->step + x * Monitor->elemSize()] = 230;
						Monitor->data[y * Monitor->step + x * Monitor->elemSize() + 1] = 230;
						if (LowPriority_Pre.data[y * Map_Now->cols + x] == 255)
							Monitor->data[y * Monitor->step + x * Monitor->elemSize() + 2] = 255;
						else
							Monitor->data[y * Monitor->step + x * Monitor->elemSize() + 2] = 180;

						keepwork = true;
						ShowCount++;

						if (EndP.x == x && EndP.y == y) {
							P_hit = true;
						}
					}
				}
			}
			Map_Now->copyTo(Map_Pre);
			RouteCount++;
			cv::threshold(LowPriority, LowPriority, 200, 255, cv::THRESH_BINARY);

#if MonitorCounter != 0
			if (ShowCount > MonitorCounter*2) {
				ShowCount = 0;
				imshow("Map_Monitor", *Monitor);
				cvWaitKey(1);
			}
#endif
		}
		return P_hit;

	}

	bool RouteFlowCalculate(cv::Point StartP, cv::Point EndP) {
		RouteOri.clear();
		bool passed = true;
		int x = 0, y = 0;
		int ShowCount = 0;

		if (RouteFlow.at<short>(EndP.y, EndP.x) > 0) {

			//putText( src , "Start", MouseClickP , 1, 1.0, Scalar(0,255,0), 1, 8, false );
			//circle(Monitor, StartP, 8, Scalar(0, 0, 255), 3, 8, 0);

			line(Monitor, StartP, StartP, cv::Scalar(0, 0, 255), 2);
			line(Monitor, EndP, EndP, cv::Scalar(0, 0, 255), 2);

			y = EndP.y;
			x = EndP.x;

			//RouteOri.push_back(EndP);

			while (y != StartP.y || x != StartP.x) {

				RouteOri.push_back(cv::Point(x, y));


				if (FindRoute(&RouteFlow, &x, &y)) {
					if (LowPriority_Original.at<char>(cv::Point(x, y)))
						line(Monitor, cv::Point(x, y), cv::Point(x, y), cv::Scalar(255, 0, 0), 1);
					else {
						line(Monitor, cv::Point(x, y), cv::Point(x, y), cv::Scalar(180, 0, 0), 1);
					}

				}
				else {
					passed = false;
					line(Monitor, cv::Point(x, y), cv::Point(x, y), cv::Scalar(0, 255, 0), 1);
					break;
				}
				///draw a object move route
				//circle(Monitor, Point(x, y), RobotSizePix / 2 - 1, Scalar(255, 0, 0), CV_FILLED, 8, 0);

				ShowCount++;
#if MonitorCounter != 0
				if (ShowCount % MonitorCounter == 0) {
					imshow("Map_Monitor", Monitor);
					cvWaitKey(1);
				}
#endif
			}
#if MonitorCounter != 0
			if (ShowCount % MonitorCounter == 0) {
				imshow("Map_Monitor", Monitor);
				cvWaitKey(1);
			}
#endif
			RouteOri.push_back(cv::Point(x, y));

		}
		else {
#if MonitorCounter != 0
			putText(Monitor, "There is no route!", cv::Point(Monitor.cols / 6, Monitor.rows / 2), 1, 3, cv::Scalar(20, 20, 255), 5, 8, false);
			imshow("Map_Monitor", Monitor);
			cvWaitKey(1);
#endif
			passed = false;
		}

		return passed;
	}
	void FindRouteProcess(cv::Point StartP, cv::Point EndP) {
		cv::Mat Map_Now;
		cv::Mat Priority;

		Map_Original.copyTo(Map_Now);
		RouteFlow = cv::Scalar(0);

		if (!AreaSearch(&Map_Now, &RouteFlow, &Monitor, StartP, EndP)) {
			printf("Point Error!\n");
		}

		bool passed = RouteFlowCalculate(StartP, EndP);

		if (passed) {
			LineDetctorFunc();
			for (int i = 0; i < RouteNew.size() - 1; i++) {
				line(Monitor, RouteNew.at(i), RouteNew.at(i + 1), cv::Scalar(0, 0, 0), 5);
				line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 255, 0), 2);
				cv::imshow("Map_Monitor", Monitor);
				cv::waitKey(1);
			}
		}
		else {
			printf("RouteFlowCalculate Error!\n");
		}
	}

	void FindRouteProcess_Block_Auto(cv::Point StartP, cv::Point EndP) {
		cv::Mat Map_Now;
		cv::Mat Priority;

		Map_Original.copyTo(Map_Now);
		RouteFlow = cv::Scalar(0);

		if (!AreaSearch(&Map_Now, &RouteFlow, &Monitor, StartP, EndP)) {
			printf("Point Error!\n");
		}

		bool passed = RouteFlowCalculate(StartP, EndP);

		//////代刚ㄧΑ
		cv::Point TestFucP;
		char MapBlockValueChar;
		Map_BlockRoute.clear();
		MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(0));
		Map_BlockRoute.push_back(MapBlockValueChar- BlockMinValue);

		RouteNew.push_back(RouteOri.at(0));
		for (int i = 1; i < RouteOri.size(); i++) {
			MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(i));
			if (MapBlockValueChar == 0) {
				MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(i - 1));
				Map_BlockValue->at<char>(RouteOri.at(i)) = MapBlockValueChar;
			}
			if (Map_BlockValue->at<char>(RouteOri.at(i - 1)) != MapBlockValueChar)
				Map_BlockRoute.push_back(MapBlockValueChar - BlockMinValue);
		}

		//////代刚ㄧΑ
		if (passed) {
			if (Map_BlockRoute.size() < 3) {

				LineDetctorFunc();
				for (int i = 0; i < RouteNew.size() - 1; i++) {
					line(Monitor, RouteNew.at(i), RouteNew.at(i + 1), cv::Scalar(0, 0, 0), 5);
					line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 255, 0), 2);
					cv::imshow("Map_Monitor", Monitor);
					cv::waitKey(1);
				}
			}
			else {

				cv::Point Start_Block_End;
				cv::Point End_Block_End;
				int blockdir = 0;

				//blockdir = Map_BlockRoute.at(0);
				Start_Block_End = LineGetCross(BlockCenter.at(Map_BlockRoute.at(0)), BlockCenter.at(Map_BlockRoute.at(1)) );
				//Start_Block_End = (0.5 * BlockCenter.at(Map_BlockRoute.at(0)) + 0.5*BlockCenter.at(Map_BlockRoute.at(1)));
				blockdir = Map_BlockRoute.size() - 1;
				End_Block_End = LineGetCross(BlockCenter.at(Map_BlockRoute.at(blockdir)), BlockCenter.at(Map_BlockRoute.at(blockdir - 1)));
				//End_Block_End = (0.5 * BlockCenter.at(Map_BlockRoute.at(blockdir)) + 0.5*BlockCenter.at(Map_BlockRoute.at(blockdir - 1)));


				routerTmp.clear();
				RouteOri.clear();
				RouteNew.clear();
				FindRouteProcess(Start_Block_End, EndP);

				routerTmp.assign(RouteNew.begin(), RouteNew.end());
				for (int i = 1; i < Map_BlockRoute.size() - 1; i++) {
					blockdir = Map_BlockRoute.at(i);
					routerTmp.push_back(BlockCenter.at(blockdir));
				}

				RouteOri.clear();
				RouteNew.clear();
				FindRouteProcess(StartP, End_Block_End);
				routerTmp.insert(routerTmp.end(), RouteNew.begin(), RouteNew.end());
				RouteNew.clear();
				RouteNew.assign(routerTmp.begin(), routerTmp.end());

				for (int i = 0; i < RouteNew.size() - 1; i++) {

					line(Monitor, RouteNew.at(i), RouteNew.at(i + 1), cv::Scalar(0, 0, 0), 3);
					line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 255, 0), 2);
					putText(Monitor, std::to_string(i), RouteNew.at(i), 1, 1.0, cv::Scalar(128, 128, 255), 1, 8, false);
					cv::imshow("Map_Monitor", Monitor);
					cv::waitKey(1);
				}

				putText(Monitor, "Start",StartP, 1, 1.0, cv::Scalar(0, 0,250), 1, 8, false);
				

				cv::imshow("Map_Monitor", Monitor);
				cv::waitKey(1);
				//cv::destroyWindow("Map_Monitor");


			}
		}

	}

	void FindRouteProcess_Block_Only(cv::Point StartP, cv::Point EndP , std::vector<cv::Point> Vector_P,int Vector_size) {
		cv::Mat Map_Now;
		cv::Mat Priority;

		Map_Original.copyTo(Map_Now);
		//RouteFlow = cv::Scalar(0);
		//if (!AreaSearch(&Map_Now, &RouteFlow, &Monitor, StartP, EndP)) {
		//	printf("Point Error!\n");
		//}

		bool passed = true;
		RouteOri.clear();
		for (int i = Vector_size - 1; i >= 0; i--) {
			RouteOri.push_back(Vector_P[i]);
		}

		//////代刚ㄧΑ
		cv::Point TestFucP;
		char MapBlockValueChar;
		Map_BlockRoute.clear();
		MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(0));
		Map_BlockRoute.push_back(MapBlockValueChar - BlockMinValue);

		RouteNew.push_back(RouteOri.at(0));
		for (int i = 1; i < RouteOri.size(); i++) {
			MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(i));
			if (MapBlockValueChar == 0) {
				MapBlockValueChar = Map_BlockValue->at<char>(RouteOri.at(i - 1));
				Map_BlockValue->at<char>(RouteOri.at(i)) = MapBlockValueChar;
			}
			if (Map_BlockValue->at<char>(RouteOri.at(i - 1)) != MapBlockValueChar)
				Map_BlockRoute.push_back(MapBlockValueChar - BlockMinValue);
		}

		//////代刚ㄧΑ
		if (passed) {
			if (Map_BlockRoute.size() < 3) {

				LineDetctorFunc();
				for (int i = 0; i < RouteNew.size() - 1; i++) {
					line(Monitor, RouteNew.at(i), RouteNew.at(i + 1), cv::Scalar(0, 0, 0), 5);
					line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 255, 0), 2);
					cv::imshow("Map_Monitor", Monitor);
					cv::waitKey(1);
				}
			}
			else {

				cv::Point Start_Block_End;
				cv::Point End_Block_End;
				int blockdir = 0;

				//blockdir = Map_BlockRoute.at(0);
				Start_Block_End = LineGetCross(BlockCenter.at(Map_BlockRoute.at(0)), BlockCenter.at(Map_BlockRoute.at(1)));
				//Start_Block_End = (0.5 * BlockCenter.at(Map_BlockRoute.at(0)) + 0.5*BlockCenter.at(Map_BlockRoute.at(1)));
				blockdir = Map_BlockRoute.size() - 1;
				End_Block_End = LineGetCross(BlockCenter.at(Map_BlockRoute.at(blockdir)), BlockCenter.at(Map_BlockRoute.at(blockdir - 1)));
				//End_Block_End = (0.5 * BlockCenter.at(Map_BlockRoute.at(blockdir)) + 0.5*BlockCenter.at(Map_BlockRoute.at(blockdir - 1)));


				routerTmp.clear();
				RouteOri.clear();
				RouteNew.clear();
				FindRouteProcess(Start_Block_End, EndP);

				routerTmp.assign(RouteNew.begin(), RouteNew.end());
				for (int i = 1; i < Map_BlockRoute.size() - 1; i++) {
					blockdir = Map_BlockRoute.at(i);
					routerTmp.push_back(BlockCenter.at(blockdir));
				}

				RouteOri.clear();
				RouteNew.clear();
				FindRouteProcess(StartP, End_Block_End);
				routerTmp.insert(routerTmp.end(), RouteNew.begin(), RouteNew.end());
				RouteNew.clear();
				RouteNew.assign(routerTmp.begin(), routerTmp.end());

				for (int i = 0; i < RouteNew.size() - 1; i++) {

					line(Monitor, RouteNew.at(i), RouteNew.at(i + 1), cv::Scalar(0, 0, 0), 3);
					line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 255, 0), 2);
					putText(Monitor, std::to_string(i), RouteNew.at(i), 1, 1.0, cv::Scalar(128, 128, 255), 1, 8, false);
					cv::imshow("Map_Monitor", Monitor);
					cv::waitKey(1);
				}

				putText(Monitor, "Start", StartP, 1, 1.0, cv::Scalar(0, 0, 250), 1, 8, false);


				cv::imshow("Map_Monitor", Monitor);
				cv::waitKey(1);
				//cv::destroyWindow("Map_Monitor");


			}
		}

	}

	cv::Point LineGetCross(cv::Point L1P1, cv::Point L1P2) {
		double Line_1_m, Line_1_c;
		double CrossPointX, CrossPointY;
		double tmp;
		cv::Point Map_dir;
		cv::Point start,end;
		tmp = (double)(L1P2.x - L1P1.x);
		if (tmp == 0)
			tmp = 0.000000001;
		Line_1_m = ((double)(L1P2.y - L1P1.y) / tmp);
		Line_1_c = L1P1.y - (Line_1_m * L1P1.x);
		bool type;
		int lenth;
		if (abs(L1P1.x - L1P2.x) > abs(L1P1.y - L1P2.y)) {
			type = true;
			lenth = abs(L1P1.x - L1P2.x);
			if (L1P1.x < L1P2.x) {
				start = L1P1;
				end = L1P2;
			}
			else {
				start = L1P2;
				end = L1P1;
			}
			char startValue ,endValue;
			endValue = Map_BlockValue->at<char>(end);
			for (int i = 1; i < lenth; i++) {
				start.x++;
				start.y = (Line_1_m * start.x) + Line_1_c;
				startValue = Map_BlockValue->at<char>(start);
				if (startValue == endValue) {
					break;
				}
			}
		}
		else{
			type = false;
			lenth = abs(L1P1.y - L1P2.y);

			if (L1P1.y < L1P2.y) {
				start = L1P1;
				end = L1P2;
			}
			else {
				start = L1P2;
				end = L1P1;
			}

			char startValue, endValue;
			endValue = Map_BlockValue->at<char>(end);
			for (int i = 1; i < lenth; i++) {
				start.y++;
				start.x = (start.y - Line_1_c)/ Line_1_m ;
				startValue = Map_BlockValue->at<char>(start);
				if (startValue == endValue) {
					break;
				}
			}

		}
		return start;
	}

	void LineDetctorFunc() {
		RouteNew.clear();
		cv::Mat LinePass;
		cv::Mat LinePassMask;
		cv::Point RectOri;
		int NewPointCount = 0;
		int startPoint = 0;
		double min, max;
		int randnnn[3];
		int searcetype = 0;

		bool hit = false;

		cv::Mat Map_Now;
		cv::Mat Map_Ori;
		cv::Mat Priority;

		Map_Original.copyTo(Map_Now);
		Map_Original_rev.copyTo(Map_Ori);

		LowPriority_Original.copyTo(Priority);
		Priority = Priority * 0.5 + Map_Ori;
		RouteNew.push_back(RouteOri.at(0));

		for (int i = 1; i < RouteOri.size(); i++) {

			if (((i - startPoint) == 1)) {
				randnnn[0] = rand() % 255;
				randnnn[1] = rand() % 255;
				randnnn[2] = rand() % 255;
				if (Priority.at<uchar>(RouteOri.at(startPoint)) == 128) {
					searcetype = 2;
				}
				else {
					searcetype = 0;
				}
				if (Priority.at<uchar>(RouteOri.at(i)) == 128) {
					searcetype++;
				}
				if (searcetype == 2) {
					Priority.at<uchar>(RouteOri.at(startPoint)) = 0;
				}
			}

			cv::Rect RectSel(RouteOri.at(startPoint), RouteOri.at(i));
			RectSel.height++;
			RectSel.width++;
			RectOri.x = RectSel.x;
			RectOri.y = RectSel.y;

			LinePassMask = cv::Mat::zeros(RectSel.size(), CV_8UC1);
			line(LinePassMask, RouteOri.at(startPoint) - RectOri, RouteOri.at(i) - RectOri, cv::Scalar(0xff), 1);

			Priority(RectSel).copyTo(LinePass, LinePassMask);

			cv::minMaxLoc(LinePass, &min, &max);

			if (searcetype == 0 || searcetype == 2) {
				if (max) {
					hit = true;
				}

			}
			else if (searcetype == 1) {
				if (max == 255 || Priority.at<uchar>(RouteOri.at(i)) == 0) {
					hit = true;
				}
			}
			else if (searcetype == 3) {
				if (max == 255) {
					hit = true;
				}
			}

			//int distance = cv::norm(RouteOri.at(startPoint) - RouteOri.at(i));
			//if (distance * 50 >= LINE_MAX_DISTANCE) {
			//	hit = true;
			//}


			if (hit) {
				i--;
				RouteNew.push_back(RouteOri.at(i));
				NewPointCount++;
				startPoint = i;
				randnnn[0] = rand() % 255;
				randnnn[1] = rand() % 255;
				randnnn[2] = rand() % 255;
				line(Monitor, RouteOri.at(i), RouteOri.at(i), cv::Scalar(randnnn[0], randnnn[1], randnnn[2]), 1);
				hit = false;
			}
			else {
				line(Monitor, RouteOri.at(i), RouteOri.at(i), cv::Scalar(randnnn[0], randnnn[1], randnnn[2]), 1);
			}
#if MonitorCounter != 0
			cv::imshow("Map_Monitor", Monitor);
			cv::waitKey(1);
#endif
		}
		RouteNew.push_back(RouteOri.at(RouteOri.size() - 1));

		PaintRouterNew();
	}

	void PaintRouterNew() {
		for (int i = 1; i < RouteNew.size(); i++) {

			line(Monitor, RouteNew.at(i - 1), RouteNew.at(i), cv::Scalar(0, 200, 200), 1);

			cv::imshow("Map_Monitor", Monitor);
			cv::waitKey(1);

		}
		for (int i = 0; i < RouteNew.size(); i++) {

			line(Monitor, RouteNew.at(i), RouteNew.at(i), cv::Scalar(0, 100, 255), 3);

			cv::imshow("Map_Monitor", Monitor);
			cv::waitKey(1);

		}
	}

};



#endif
