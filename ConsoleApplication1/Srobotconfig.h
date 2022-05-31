
#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_

#include "Utils.h"

const double PI = 3.1415926;
const double L1 = 250;
const double L2 = 250;

using namespace Eigen;

namespace SRobot
{
    extern double mTransMatrix[16];
    extern bool mConfig;
    extern Vector3d w[4], q[4];
    extern Vector4d q_[4];
    extern Matrix<double, 6, 1> kexi[4];
    extern Matrix4d gst_0;

    void Init_Scara();

	//Inverse kinematics solution
	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll);
	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4);
	
	//Forward kinematics solution
	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4);
	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll);

	//Inverse kinematics and Forward kinematics method function
	void robotBackward(const double* TransVector, bool config, double* theta);
	void robotForward(const double* q, double* TransVector, bool config);
}

#endif
