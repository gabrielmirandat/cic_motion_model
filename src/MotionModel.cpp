#include <MotionModel.h>
#include <math.h>
#include <ctime>

MotionModel::MotionModel()
: psXtm1_ (NUM_PARTICLES), psXt_ (NUM_PARTICLES)
{
    alpha1_ = 0.01; // angular error
    alpha2_ = 0.01; // translational error
    alpha3_ = 0.01; // translational error
    alpha4_ = 0.01; // angular error

    old_control_ << 0,0,0;
    new_control_ << 0,0,0;

    // seed e gerador usados nas funções de probabilidade do c++11
    seed_ = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed_);
    generator_ = generator;
}

// corrige o angulo, trazendo de volta p faixa [-pi, pi)
inline double truncAngle (double theta) {
    if (theta >= M_PI)
        return (theta - 2.0*M_PI);
    else if (theta < -M_PI)
        return (theta + 2*M_PI);
    else return theta;
}

void MotionModel::setAlpha(double alpha1, double alpha2, double alpha3, double alpha4)
{
    alpha1_ = alpha1; // angular error
    alpha2_ = alpha2; // translational error
    alpha3_ = alpha3; // translational error
    alpha4_ = alpha4; // angular error
}

void MotionModel::sample_motion_model_odometry()
{
    // std::cout << "sample_motion_model_odometry" << std::endl;
    double delta_rot_1 = atan2(control_.getXt(1) - control_.getXtm1(1),control_.getXt(0)
                               - control_.getXtm1(0)) - control_.getXtm1(2);
    double delta_trans = sqrt((control_.getXtm1(0) - control_.getXt(0))
                               *(control_.getXtm1(0) - control_.getXt(0))
                               + (control_.getXtm1(1) - control_.getXt(1))
                               *(control_.getXtm1(1) - control_.getXt(1)));
    double delta_rot_2 = control_.getXt(2) - control_.getXtm1(2) - delta_rot_1;

    if(std::isnan(delta_rot_1) or std::isnan(delta_trans) or std::isnan(delta_rot_2))
        return;

    // traz os angulos para a faixa [-pi, pi) radianos, caso estejam fora
    delta_rot_1 = truncAngle (delta_rot_1);
    delta_rot_2 = truncAngle (delta_rot_2);

    double delta_rot_1_corr = delta_rot_1 - sample(alpha1_*fabs(delta_rot_1) + alpha2_*delta_trans);
    double delta_trans_corr = delta_trans - sample(alpha3_*delta_trans + alpha4_*(fabs(delta_rot_1) + fabs(delta_rot_2)));
    double delta_rot_2_corr = delta_rot_2 - sample(alpha1_*fabs(delta_rot_2) + alpha2_*delta_trans);

    new_state_(0) = state_(0) + delta_trans_corr * cos (state_(2) + delta_rot_1_corr);
    new_state_(1) = state_(1) + delta_trans_corr * sin (state_(2) + delta_rot_1_corr);
    new_state_(2) = truncAngle (state_(2) + delta_rot_1_corr + delta_rot_2_corr);
    // std::cout << "new_state_1 " << new_state_ << std::endl;
    
}

double MotionModel::sample(double b)
{
    // faz o sample com funções do c++11
    normal_distribution<double> errorDist(0.0, b);
    return errorDist(generator_);
}

void MotionModel::setOldControl(double x, double y, double ori)
{
    old_control_(0) = x;
    old_control_(1) = y;
    old_control_(2) = ori;
}

void MotionModel::updateControl(double x, double y, double ori)
{
    old_control_ = new_control_;

    new_control_(0) = x;
    new_control_(1) = y;
    new_control_(2) = ori;

}

void MotionModel::run()
{

    Eigen::VectorXd control_vector(6);
    control_vector << old_control_, new_control_;
    control_.set(control_vector);

    control_.print();

    // constroi o novo conjunto de particulas
    for (int m = 0; m < NUM_PARTICLES; ++m) {
        // particula antiga
        state_ = psXtm1_[m];
        // sorteia nova particula de acordo com o algoritmo que fizemos
        sample_motion_model_odometry();
        // guarda a nova particula
        psXt_[m] = new_state_;
        // atualiza os dados para o gnuplot
        viz_.setDataX(m, new_state_(0));
        viz_.setDataY(m, new_state_(1));
        // std::cout << "new_state_2 " << new_state_ << std::endl;
    }

    viz_.showPlot();

    // agora o estado atual vai ser o novo estado passado
    psXtm1_ = psXt_;
}
