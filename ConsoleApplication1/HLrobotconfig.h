
#ifndef HLROBOTCONFIG_H
#define HLROBOTCONFIG_H
#pragma once
#include "eigen3/Eigen/Dense"

#define PI 3.1415926
namespace HLRobot
{ 
	//double qlimP[6] = { 170 * PI / 180,120 * PI / 180,48 * PI / 180, 180 * PI / 180, 120 * PI / 180,360 * PI / 180 };
	//double qlimN[6] = { -170 * PI / 180,-120 * PI / 180,-228 * PI / 180, -180 * PI / 180, -120 * PI / 180,-360 * PI / 180 };
	//constexpr double mTransMatrix[16];
	//constexpr bool mConfig[3] = {1, 1, 1};
	//constexpr bool Turns[6] = {1,1,1,1,1,1};


	void robotBackwardHJQ(const double* TransVector, bool* config, double* theta);
	void robotBackwardHJQ(const Vector3d& pos, const Vector3d& ang, bool* config, double* theta);
	void robotForwardHJQ(const double* q, const double* Tool, double* TransVector, bool* config, bool* turns);
	//求运动学逆解，获得各个关节的角度
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4, double &angle5, double &angle6);
    //使用当前位置的XYZRPY计算，位形矩阵
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll, bool* config);
}

#endif
