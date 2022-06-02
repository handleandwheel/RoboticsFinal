#include<iostream>
#include<winsock.h>
#include <conio.h>
#include <vector>
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"
#include "MotionPlan.h"
#include "RobotCommunication.h"

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "WS2_32.lib")

using namespace std;

int main()
{
	//PosStruct homePoint = {};
	//PosStruct safePoint = {};
	//PosStruct warePoint = {};
	//PosStruct throwPoint = {};
	//PosStruct endPoint = {};
	//vector<PosStruct> ctrlPoints = { homePoint, safePoint, warePoint, endPoint };
	//HLMotionPlan motion_plan;
	//motion_plan.SetSampleTime();
	//motion_plan.SetLinearParam();
	//motion_plan.SetAngularParam();
	//motion_plan.SetCtrlPoints();
	//motion_plan.GetPlanPoints();
	
	RobotCommunication robo_com;
	robo_com.initialize();

	FtpControl::Upload("192.168.10.101", "data", "data.txt", "data.txt");

	robo_com.communicate("[7# PPB.Enable 1,1]", "[PPB_Enable]", 200);
	robo_com.communicate("[8# Robot.Frame 1,1]", "[Frame2]", 200);
	robo_com.communicate("[9# PPB.ReadFile 1,/data/data.txt]", "[ReadFile]", 500);
	robo_com.communicate("[10# PPB.J2StartPoint 1,0,1]", "[ToStartPoint]", 1000);
	robo_com.communicate("[6# PPB.Run 1]", "[Run]", 1000);

	return 0;
}