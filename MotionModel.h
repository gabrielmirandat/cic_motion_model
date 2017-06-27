#ifndef _MOTION_MODEL_H_
#define _MOTION_MODEL_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

using namespace std;

class Control;

class MotionModel
{
private:
    double alpha1_, alpha2_, alpha3_, alpha4_;

    double sample(double b);
public:
    MotionModel();
    Eigen::Vector3d sample_motion_model_odometry(Control u, Eigen::Vector3d state);
};

class Control
{
private:
    Eigen::VectorXd u_t_;
    // Eigen::VectorXd u_t_T_;

public:
    Control()
    : u_t_(6)
    {
        // u_t_ = (x_bar_tm1_ x_bar_t_)T
        Eigen::Vector3d x_bar_tm1(0.0,0.0,0.0);
        Eigen::Vector3d x_bar_t(0.0,0.0,0.0);

        // concatenate the two vectors
        u_t_ << x_bar_tm1, x_bar_t;
    }

    Eigen::VectorXd getVec(){return u_t_;}
    double getXtm1(int index) {return static_cast<double>(u_t_[index]);}
    double getXt(int index) {return static_cast<double>(u_t_[index+3]);}

    void set(int index, float value) {u_t_[index] = value;}

    void print()
    {
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        cout << "u_t_= [" << u_t_.rows() << "," << u_t_.cols() << "]" << endl;
        cout << u_t_.format(CleanFmt) << endl;
    }

};

#endif //_MOTION_MODEL_H_
