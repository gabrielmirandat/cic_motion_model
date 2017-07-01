#include <MotionModel.h>
#include <math.h>
#include <ctime>

MotionModel::MotionModel()
{
    alpha1_ = 0.01; // translational error
    alpha2_ = 0.01; // translational error
    alpha3_ = 0.01; // angular error
    alpha4_ = 0.01; // angular error
}

// in
// control u_t
// pose x_tm1
Eigen::Vector3d MotionModel::sample_motion_model_odometry(Control u, Eigen::Vector3d state)
{
    // cout << sample(1.0) << endl;
    // cout << sample(0.5) << endl;
    // cout << sample(0.02) << endl;

    double delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);
    double delta_trans = sqrt((u.getXtm1(0) - u.getXt(0))*(u.getXtm1(0) - u.getXt(0))
                           + (u.getXtm1(1) - u.getXt(1))*(u.getXtm1(1) - u.getXt(1)));
    double delta_rot_2 = u.getXt(2) - u.getXtm1(2) - delta_rot_1;


    double delta_rot_1_corr = delta_rot_1 - sample(alpha1_*fabs(delta_rot_1) + alpha2_*delta_trans);
    double delta_trans_corr = delta_trans - sample(alpha3_*delta_trans + alpha4_*(fabs(delta_rot_1) + fabs(delta_rot_2)));
    double delta_rot_2_corr = delta_rot_2 - sample(alpha1_*fabs(delta_rot_2) + alpha2_*delta_trans);

    Eigen::Vector3d new_state;
    new_state[0] = state[0] + delta_trans_corr * cos (state[2] + delta_rot_1_corr);
    new_state[1] = state[1] + delta_trans_corr * sin (state[2] + delta_rot_1_corr);
    new_state[2] = state[2] + delta_rot_1_corr + delta_rot_2_corr;

    return new_state;
}

double MotionModel::sample(double b)
{
    assert(b>0.0);
    srand(time(NULL));
    double sample_value = 0.0;
    for(int i=0; i < 12; i++)
    {
      double f = (double)rand() / RAND_MAX;
      sample_value += (-1)*b + f * (b - (-1)*b);
    }
    return sample_value/2;
}

int main()
{
    MotionModel mm;
    Control u;

    Eigen::Vector3d state, new_state;
    Eigen::VectorXd control(6);
    Eigen::Vector3d new_control, old_control;

    state =       Eigen::Vector3d(10.0,
                                  10.0,
                                  0.785398 /*45º*/);

    old_control = Eigen::Vector3d(0.1,
                                  0.1,
                                  0.0174533 /*1º*/);

    new_control = Eigen::Vector3d(0.1,
                                  0.1,
                                  0.0174533 /*1º*/);

    control << old_control, new_control;
    u.set(control);

    cout << "INITIAL_STATE" << endl;
    cout << state << "\n" << endl;

    cout << "CONTROL" << endl;
    u.print();

    new_state = mm.sample_motion_model_odometry(u, state);
    cout << "NEW_STATE" << endl;
    cout << new_state << endl;

    return 0;
}
