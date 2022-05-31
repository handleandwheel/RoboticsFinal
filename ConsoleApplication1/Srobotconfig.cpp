
#include "Srobotconfig.h"
#include "iostream"
#include "eigen3/Eigen/Dense"
#include "Utils.h"
#include <math.h>
#define M_PI 3.14159265
using namespace std;
using namespace Eigen;

namespace SRobot
{
    //初始化TransMatrix
    double mTransMatrix[16] {0};

    //只使用一种姿态
    bool mConfig = 1;
    // Interior state
    Vector3d w[4], q[4];
    Vector4d q_[4];
    Matrix<double, 6, 1> kexi[4];
    Matrix4d gst_0;

    void Init_Scara(){
        w[0] << 0, 0, 1;
        w[1] << 0, 0, 1;
        w[2] << 0, 0, 0;
        w[3] << 0, 0, -1;
        q[0] << 0, 0, 0;
        q[1] << L1, 0, 0;
        q[2] << L1+L2, 0, 0;
        q[3] << L1+L2, 0, 0;
        for (int i=0; i<4; i++) {
            q_[i] << q[i], 1;
            if(i==2){
                kexi[i] << 0,0,-1,0,0,0;
            }else{
                Vector3d temp = -w[i].cross(q[i]);
                kexi[i].block<3, 1>(0, 0) = temp;
                kexi[i].block<3, 1>(3, 0) = w[i];
            }
        }
        gst_0 = Utils::TransformRP2G(Matrix3d::Identity(), {L1+L2, 0, 0});
        gst_0(1, 1) = -1;
        gst_0(2, 2) = -1;
    }

	void SetRobotEndPos(double x, double y, double z, double yaw, double pitch, double roll){
        Matrix4d gst_temp, gst_temp_trans;
        Vector3d w_y, w_z;
        Matrix3d w_y_hat, w_z_hat, rotate_matrix;
        w_y << 0, 1, 0; w_y_hat = Utils::skew(w_y);
        w_z << 0, 0, 1; w_z_hat = Utils::skew(w_z);
        rotate_matrix =Utils::exp_r(w_z_hat, yaw/180.0*M_PI)
                * Utils::exp_r(w_y_hat, pitch/180.0*M_PI)
                * Utils::exp_r(w_z_hat, roll/180.0*M_PI);
        gst_temp = Utils::TransformRP2G(rotate_matrix, {x, y, z});
        gst_temp_trans = gst_temp.transpose();
        for (int i = 0; i < 16; i++){
            mTransMatrix[i] = gst_temp_trans(i);
        }


	}

	void GetJointAngles(double &angle1, double &angle2, double &angle3, double &angle4){
        double theta[6] = { 0 };
        robotBackward(mTransMatrix, mConfig, theta);
        angle1 = theta[0];
        angle2 = theta[1];
        angle3 = theta[2];
        angle4 = theta[3];
	}

	void SetRobotJoint(double angle1, double angle2, double angle3, double angle4){
        Matrix4d exp_g[4];
        exp_g[0] = Utils::exp_g(kexi[0], angle1);
        exp_g[1] = Utils::exp_g(kexi[1], angle2);
        exp_g[2] = Utils::TransformRP2G(Eigen::Matrix3d::Identity(), {0,0,-angle3});
        exp_g[3] = Utils::exp_g(kexi[3], angle4); 

        Matrix4d gst_theta;
        gst_theta = exp_g[0]*exp_g[1]*exp_g[2]*exp_g[3]*gst_0;
        for (int i = 0; i < 16; i++){
            mTransMatrix[i] = gst_theta(i);
        }
	}

	void GetJointEndPos(double &x, double &y, double &z, double &yaw, double &pitch, double &roll){
        Matrix4d gst_theta;
        for (int i = 0; i < 16; i++)
        {
            gst_theta(i) = mTransMatrix[i];
        }
        x = gst_theta(0, 3);
        y = gst_theta(1, 3);
        z = gst_theta(2, 3);

        pitch = atan2(gst_theta(0, 2), sqrt(gst_theta(1, 2) * gst_theta(1, 2) + gst_theta(2, 2) * gst_theta(2, 2)))*180.0/PI;
        roll= atan2(-gst_theta(1,2)/cos(pitch), gst_theta(2, 2) / cos(pitch)) * 180.0 / PI;
        yaw = atan2(-gst_theta(0, 1) / cos(pitch), gst_theta(0, 0) / cos(pitch)) * 180.0 / PI;

	}


	/********************************************************************
	ABSTRACT:	机器人逆运动学

	INPUTS:		T[16]:	位姿矩阵，其中长度距离为米

				config：姿态，六轴机器人对应有8种姿态（即对应的逆运动学8个解），
				Scara机器人有2个解，Delta机器人有一个解，
				为了安全，实验室中我们只计算一种即可。config用来作为选解的标志数。

	OUTPUTS:    theta[6] 6个关节角, 单位为弧度

	RETURN:		<none>
	***********************************************************************/
	void robotBackward(const double* TransVector, bool mconfig, double* theta){
		Matrix4d gst_temp_trans, gst_temp;
        for (int i = 0; i < 16; i++){
            gst_temp_trans(i) = TransVector[i];
        }
        gst_temp = gst_temp_trans.transpose();
        theta[4]=0; theta[5]=0;

        // theta3
        theta[2] = -gst_temp(2, 3);

        // theta2
        Matrix4d exp_g_3;
        exp_g_3 = Utils::TransformRP2G(Eigen::Matrix3d::Identity(), {0,0,-theta[2]});
        Matrix4d g1;
        g1 = gst_temp*gst_0.inverse()*exp_g_3.inverse();


        Vector4d temp_value;
        temp_value = g1*q_[2] - q_[0];
        double delta = sqrt(temp_value.transpose()*temp_value);
        theta[1] = Utils::Subproblem_3(q[2], q[0], q[1], w[1], delta);


        // theta1
        Vector4d temp_p_ = Utils::exp_g(kexi[1], theta[1])*q_[2];
        Vector4d temp_q_ = g1*q_[2];
        theta[0] = Utils::Subproblem_1(temp_p_.segment<3>(0),
                temp_q_.segment<3>(0),q[0], w[0]);

        // theta4
        Matrix4d g2;
        g2 = exp_g_3.inverse()*Utils::exp_g(kexi[1], theta[1]).inverse()
                *Utils::exp_g(kexi[0], theta[0]).inverse()
                *gst_temp*gst_0.inverse();
        temp_q_ = g2*q_[1];
        theta[3] = Utils::Subproblem_1(q[1], temp_q_.segment<3>(0), q[3], w[3]);

        // to degree
        theta[0] = theta[0] * 180.0 / M_PI;
        theta[1] = theta[1] * 180.0 / M_PI;
        theta[3] = theta[3] * 180.0 / M_PI;
	}

	/********************************************************************
	ABSTRACT:	机器人正运动学
	
	INPUTS:		q[6]: 6个关节角, 单位为弧度
	
	OUTPUTS:	config用来作为选解的标志数。

				TransVector[16] : 刚体变换矩阵，也就是末端的位姿描述，其中长度距离为米
	
	RETURN:		<none>
	***********************************************************************/
	void robotForward(const double* q, double* TransVector, bool mconfig){
		Matrix4d gst_theta;
        Matrix4d exp_g[4];
        for (int i=0; i<4; i++) {
            if(i==2){
                exp_g[i] = Utils::TransformRP2G(Eigen::Matrix3d::Identity(), {0,0,-q[i]*180/M_PI});
            }else{
                exp_g[i] = Utils::exp_g(kexi[i], q[i]);
            }
        }
        gst_theta = exp_g[0]*exp_g[1]*exp_g[2]*exp_g[3]*gst_0;

        Matrix4d gst_theta_trans;
        gst_theta_trans = gst_theta.transpose();
        for (int i = 0; i < 16; i++){
            TransVector[i] = gst_theta_trans(i);
        }
	}
}
