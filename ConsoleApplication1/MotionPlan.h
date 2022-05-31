#pragma once
#include <vector>
#include "eigen3/Eigen/Core"
#include "iostream"
using namespace std;
using namespace Eigen;

struct PosStruct
{
	double x;				// mm
	double y;				// mm
	double z;				// mm
	double yaw;				// deg
	double pitch;			// deg
	double roll;			// deg
};

class HLMotionPlan
{
public:
	HLMotionPlan();
	~HLMotionPlan();

	void SetSampleTime(const double& sampleTime);
	void SetPlanPoints(const vector<PosStruct> &points);
	void SetLinearParam(const double& vel, const double& acc, const double& dec);
	void SetAngularParam(const double& ang_vel, const double& ang_acc, const double& ang_dec);
	void GetPlanPoints(const char* filename);
private:
	void PlanSegment(const PosStruct& startPoint, const PosStruct& endPoint, const ofstream& outfile);
	void PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints);
	void PlanTriangle(const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints);
	vector<PosStruct> ctrlPoints;				// way points in cartesian frame
	double mSampleTime;							// sample time, sencond, sample time in the robot is 0.001s
	double mVel;								// max linear velocity, mm/s
	double mAcc;								// max linear acceleration, mm/s/s
	double mDec;								// max linear deceleration, mm/s/s
	double mAngVel;								// max angular velocity, deg/s
	double mAngAcc;								// max angular acceleration, deg/s/s
	double mAngDec;								// max angular deceleration, deg/s/
};