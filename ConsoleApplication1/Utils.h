//
// Created by Everlasting on 2022/5/11.
//

#ifndef ROBOTICS_UTILS_H
#define ROBOTICS_UTILS_H

#include "eigen3/Eigen/Dense"

class Utils{
public:
    static Eigen::Matrix3d skew(Eigen::Vector3d vec){
        Eigen::Matrix3d temp;
        temp.setZero();
        temp <<        0, -vec.z(),  vec.y(),
                vec.z(),        0, -vec.x(),
                -vec.y(),  vec.x(),         0;
        return temp;
    }

    static Eigen::Matrix4d TransformRP2G(Eigen::Matrix3d R, Eigen::Vector3d P){
        Eigen::Matrix4d temp;
        temp.setZero();
        temp.block<3, 3>(0, 0) = R;
        temp.block<3, 1>(0, 3) = P;
        temp(3, 3) = 1;
        return temp;
    }

    static Eigen::Matrix3d exp_r(Eigen::Matrix3d w_hat, double theta){
        Eigen::Matrix3d temp;
        temp.setZero();
        temp = Eigen::Matrix3d::Identity() + w_hat * sin(theta)
                + w_hat * w_hat * (1 - cos(theta));
        return temp;
    }

    /*! theta represents by rad */
    static Eigen::Matrix4d exp_g(Eigen::Matrix<double, 6, 1> kexi, double theta){
        Eigen::Matrix4d temp;
        temp.setZero();
        Eigen::Vector3d v, w, p;
        v << kexi(0), kexi(1), kexi(2);
        w << kexi(3), kexi(4), kexi(5);
        Eigen::Matrix3d w_hat = skew(w);
        Eigen::Matrix3d exp_w = exp_r(w_hat, theta);
        p = (Eigen::Matrix3d::Identity()-exp_w)*w_hat*v
                + w*w.adjoint()*v*theta;
        temp = TransformRP2G(exp_w, p);
        return temp;
    }

    static double Subproblem_1(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r, Eigen::Vector3d w){
        double temp_theta;
        Eigen::Vector3d u = p-r;
        Eigen::Vector3d v = q-r;
        Eigen::Vector3d u_ = (Eigen::Matrix3d::Identity()-w*w.transpose())*u;
        Eigen::Vector3d v_ = (Eigen::Matrix3d::Identity()-w*w.transpose())*v;
        temp_theta = atan2(w.transpose()*(u_.cross(v_)), u_.transpose()*v_);
        return temp_theta;
    }

    static double Subproblem_3(Eigen::Vector3d p, Eigen::Vector3d q, Eigen::Vector3d r,
                               Eigen::Vector3d w, double delta){
        double temp_theta;
        Eigen::Vector3d u = p-r;
        Eigen::Vector3d v = q-r;
        Eigen::Vector3d u_ = (Eigen::Matrix3d::Identity()-w*w.transpose())*u;
        Eigen::Vector3d v_ = (Eigen::Matrix3d::Identity()-w*w.transpose())*v;
        double theta0 = atan2(w.transpose()*(u_.cross(v_)), u_.transpose()*v_);

        double u_2 = u_.transpose() * u_;
        double v_2 = v_.transpose() * v_;

        temp_theta = theta0 - acos((u_2+v_2-delta*delta)/(2* sqrt(u_2*v_2)));

        return temp_theta;

    }

};


#endif //ROBOTICS_UTILS_H
