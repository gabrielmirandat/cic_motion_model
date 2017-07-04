#ifndef _MOTION_MODEL_H_
#define _MOTION_MODEL_H_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Dense>

#include <random>
#include <chrono>
#include <thread>
#include <boost/tuple/tuple.hpp>
#include "gnuplot-iostream.h"

using namespace std;

// numero de particulas
#define NUM_PARTICLES 1000

class Control
{
private:
    Eigen::VectorXd u_t_;

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
    void set(Eigen::VectorXd control_vector) {u_t_ = control_vector;}

    void print()
    {
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
        cout << "u_t_= [" << u_t_.rows() << "," << u_t_.cols() << "]" << endl;
        cout << u_t_.format(CleanFmt) << "\n" << endl;
    }

};

class Vizualization
{
private:
    // gnuplot
    Gnuplot gp_;

    // vetores de dados que o gnuplot reconhece
    std::vector<double> filter_data_x_, filter_data_y_;

    void configure()
    {
        // configura o plot
        gp_ << "set xrange [-50:50]\n"
               "set yrange [-50:50]\n"
               "set style fill transparent solid 0.1 noborder\n"
               "set style circle radius 0.1\n"
               "set title 'Location'\n";

        // faz o plot
        gp_ << "plot '-' with circles lc rgb 'black'\n";
        gp_.send1d(boost::make_tuple(filter_data_x_, filter_data_y_));
        // espera 200ms (no ROS isso ja acontece)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

public:
    Vizualization()
    : filter_data_x_ (NUM_PARTICLES), filter_data_y_ (NUM_PARTICLES)
    {
        configure();
    }

    void setDataX(int index, double value){filter_data_x_[index] = value;}
    void setDataY(int index, double value){filter_data_y_[index] = value;}
    void showPlot()
    {
        // plota de novo os dados com o gnuplot e espera 200ms
        gp_ << "plot '-' with circles lc rgb 'black'\n";
        gp_.send1d(boost::make_tuple(filter_data_x_, filter_data_y_));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
};

class MotionModel
{
private:
    double alpha1_, alpha2_, alpha3_, alpha4_;
    Control control_;
    Eigen::Vector3d old_control_, new_control_;
    Eigen::Vector3d state_, new_state_;

    // conjuntos de particulas representando os estados passado e atual
    vector<Eigen::Vector3d> psXtm1_, psXt_;

    // vizualize
    Vizualization viz_;

    // samples
    double seed_;
    default_random_engine generator_;

    double sample(double b);
    void sample_motion_model_odometry();
public:
    MotionModel();

    void setAlpha(double alpha1, double alpha2, double alpha3, double alpha4);
    void setOldControl(double x, double y, double ori);
    void updateControl(double x, double y, double ori);
    void run();

};

#endif //_MOTION_MODEL_H_
