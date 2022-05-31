#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "Srobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace std;
using namespace SRobot;
using namespace Eigen;

HLMotionPlan::HLMotionPlan()
{
	ctrlPoints.clear();

	mSampleTime = 0.001;
	mVel = 0; mAcc = 0; mDec = 0;
	mAngVel = 0; mAngAcc = 0; mAngDec = 0;
}

HLMotionPlan::~HLMotionPlan() {}

void HLMotionPlan::SetSampleTime(const double& sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
	cout << "[Planning]\tsample time is set to: " << sampleTime << "s" << endl;
}

void HLMotionPlan::SetLinearParam(const double& vel, const double& acc, const double& dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
	cout << "[Planning]\tmax linear velocity is set to : " << mVel << "m/s" << endl;
	cout << "[Planning]\tmax linear acceleration is set to : " << mAcc << "m/s/s" << endl;
	cout << "[Planning]\tmax linear deceleration is set to : " << mDec << "m/s/s" << endl;
}

void HLMotionPlan::SetAngularParam(const double& ang_vel, const double& ang_acc, const double& ang_dec)
{
	mAngVel = ang_vel;
	mAngAcc = ang_acc;
	mAngDec = ang_dec;
	cout << "[Planning]\tmax angular elocity is set to : " << mAngVel << "deg/s" << endl;
	cout << "[Planning]\tmax angular acceleration is set to : " << mAngAcc << "deg/s/s" << endl;
	cout << "[Planning]\tmax angular deceleration is set to : " << mAngDec << "deg/s/s" << endl;
}

void HLMotionPlan::SetPlanPoints(const vector<PosStruct>& points)
{
	ctrlPoints = points;
}

void HLMotionPlan::PlanSegment(const PosStruct& startPoint, const PosStruct& endPoint, const ofstream& file)
{
	Vector3d startPos(startPoint.x, startPoint.y, startPoint.z);
	Vector3d startAng(startPoint.yaw, startPoint.pitch, startPoint.roll);
	Vector3d endPos(endPoint.x, endPoint.y, endPoint.z);
	Vector3d endAng(endPoint.yaw, endPoint.pitch, endPoint.roll);
	vector<Vector3d> wayPos;
	vector<Vector3d> wayAng;
	// triangle linear velocity
	if ((endPos - startPos).norm() < mVel * mVel * (mAcc + mDec) / (2 * mAcc * mDec))
		PlanTriangle(mAcc, mDec, startPos, endPos, wayPos);
	// trapezoidal linear velocity
	else
		PlanTrapezoidal(mVel, mAcc, mDec, startPos, endPos, wayPos);
	// triangle angular velocity
	if ((endAng - startAng).norm() < mAngVel * mAngVel * (mAngAcc + mAngDec) / (2 * mAngAcc * mAngDec))
		PlanTriangle(mAngAcc, mAngDec, startAng, endAng, wayAng);
	// trapezoidal angular velocity
	else
		PlanTrapezoidal(mAngVel, mAngAcc, mAngDec, startAng, endAng, wayAng);
	Matrix<double, 6, 1> wayPoint;
	for (int i = 0; i < max(wayPos.size(), wayAng.size()); i++)
	{
		wayPoint.topLeftCorner(3, 1) = (i < wayPos.size()) ? wayPos[i] : wayPos[wayPos.size() - 1];
		wayPoint.bottomLeftCorner(3, 1) = (i < wayAng.size()) ? wayAng[i] : wayAng[wayAng.size() - 1];
		// wayPoint(0, 1, 2, 3, 4, 5) = x, y, z, yaw, pitch, roll;
		// TODO: inverse kinematics
	}
	// TODO: put the points into file without the start and end points
}

void HLMotionPlan::GetPlanPoints(const char* filename)
{
	ofstream outfile;
	outfile.open(filename);
	for (int i = 0; i < ctrlPoints.size() - 1; i++)
	{
		// TODO: put the start points into file
		PlanSegment(ctrlPoints[i], ctrlPoints[i + 1], outfile);
	}
	// TODO: put the end points into file
	outfile.close();
}

void HLMotionPlan::PlanTriangle(const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints)
{
	// the result doesn't include the start and end point
	Vector3d direction = (endPoint - startPoint) / (endPoint - startPoint).norm();
	double time1 = sqrt(2 * (endPoint - startPoint).norm() * mDec / (mAcc + mDec) / mAcc);
	double time2 = sqrt(2 * (endPoint - startPoint).norm() * mAcc / (mAcc + mDec) / mDec);
	double time = 0.0;
	while (time < time1)
	{
		time += mSampleTime;
		wayPoints.push_back(startPoint + direction * 0.5 * mAcc * time * time);
	}
	Vector3d midPoint = startPoint + direction * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(midPoint);
	double vel = mAcc * time1;
	time = 0.0;
	while (time < time2)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * (vel * time - 0.5 * mDec * time * time));
	}
}

void HLMotionPlan::PlanTrapezoidal(const double& mVel, const double& mAcc, const double& mDec, const Vector3d& startPoint, const Vector3d& endPoint, vector<Vector3d>& wayPoints)
{
	// the result doesn't include the start and end point
	Vector3d direction = (endPoint - startPoint) / (endPoint - startPoint).norm();
	double time1 = mVel / mAcc;
	double time3 = mVel / mDec;
	double time2 = ((endPoint - startPoint).norm() - 0.5 * (mAcc * time1 * time1 + mDec * time2 * time2)) / mVel;
	double time = 0.0;
	while (time < time1)
	{
		time += mSampleTime;
		wayPoints.push_back(startPoint + direction * 0.5 * mAcc * time * time);
	}
	Vector3d midPoint = startPoint + direction * 0.5 * mAcc * time1 * time1;
	wayPoints.push_back(midPoint);
	time = 0.0;
	while (time < time2)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * mVel * time);
	}
	midPoint = midPoint + direction * mVel * time1;
	wayPoints.push_back(midPoint);
	time = 0.0;
	while (time < time3)
	{
		time += mSampleTime;
		wayPoints.push_back(midPoint + direction * (mVel * time - 0.5 * mDec * time * time));
	}
}