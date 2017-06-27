#include <MotionModel.h>
#include <math.h>
#include <ctime>

MotionModel::MotionModel()
{
    alpha1_ = 0.0;
    alpha2_ = 0.0;
    alpha3_ = 0.0;
    alpha4_ = 0.0;
}

// in
// control u_t
// pose x_tm1
Eigen::Vector3d MotionModel::sample_motion_model_odometry(Control u, Eigen::Vector3d state)
{
    u.print();
    double delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);
    double delta_trans = sqrt((u.getXtm1[0] - u.getXt[0])*(u.getXtm1[0] - u.getXt[0])
                           + (u.getXtm1[1] - u.getXt[1])*(u.getXtm1[1] - u.getXt[1]));
    double delta_rot_2 = u.getXt(2) - u.getXtm1(2) - delta_rot_1;


    double delta_rot_1_corr = delta_rot_1 - sample(alpha1_*fabs(delta_rot_1) + alpha2_*delta_trans);
    double delta_trans_corr = delta_trans - sample(alpha3_*delta_trans + alpha4_*(fabs(delta_rot_1) + fabs(delta_rot_2)));
    double delta_rot_2_corr = delta_rot_2 - sample(alpha1_*fabs(delta_rot_2) + alpha2_*delta_trans);

    Eigen::Vector3d newstate;
    newstate[0] = state[0] + delta_trans_corr * cos (state[2] + delta_rot_1_corr);
    newstate[1] = state[1] + delta_trans_corr * sin (state[2] + delta_rot_1_corr);
    newstate[2] = state[2] + delta_rot_1_corr + delta_rot_2_corr;

    return new_state;
}

double MotionModel::sample(double b)
{
    srand(time(NULL));
    double sample_value;
    for(int i = 0; i < 12; ++i)
        sample_value += (-1) + (static_cast<double>(rand()) / RAND_MAX) * (1 - (-1));

    return sample_value/b;
}

double fRand(double fMin, double fMax)
{
    double f = static_cast<double>(rand()) / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

int main()
{
    // u.print();
    // u.set(3,2.0);
    // u.print();

    MotionModel mm;
    Control u;
    Eigen::Vector3d state(0.0,0.0,0.0);

    mm.sample_motion_model_odometry(u, state);

    return 0;
}
