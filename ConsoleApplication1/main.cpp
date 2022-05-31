#include<iostream>
#include<winsock.h>
#include <conio.h>
#include "ftp/FtpControl.h"
#include "eigen3/Eigen/Core"
#include "MotionPlan.h"
#include "RobotCommunication.h"

#pragma comment(lib,"ws2_32.lib")
#pragma comment(lib, "WS2_32.lib")

using namespace std;

int main()
{
	//PosStruct start_pos = { 337.98, 48.150, -17.35, 0, 180, -154.08 };
	//PosStruct end_pos = { 337.99, -30.0, -17.35, 0.0, 180.0, -154.08 };
	//HLMotionPlan motion_plan;
	//motion_plan.SetSampleTime(0.001);
	//motion_plan.SetProfile(0.3, 1.0, 1.0);
	//motion_plan.SetPlanPoints(start_pos, end_pos);
	//motion_plan.GetPlanPoints_line();
	// TODO: get the control points and generate the path

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