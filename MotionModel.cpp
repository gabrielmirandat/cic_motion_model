#include <MotionModel.h>
#include <math.h>

MotionModel::MotionModel()
{
}

// in
// control u_t
// pose x_tm1
void MotionModel::sample_motion_model_odometry(Control u, Eigen::Vector3f state)
{
    u.print();
    float delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);


    Eigen::Vector3d newstate;
    newstate[0] = state[0] + delta_trans_corr * cos (state[2] + delta_rot_1_corr);
    newstate[1] = state[1] + delta_trans_corr * sin (state[2] + delta_rot_1_corr);
    newstate[2] = state[2] + delta_rot_1_corr + delta_rot_2_corr;

}

int main()
{
    // u.print();
    // u.set(3,2.0);
    // u.print();

    MotionModel mm;
    Control u;
    Eigen::Vector3f state(0.0,0.0,0.0);

    mm.sample_motion_model_odometry(u, state);

    return 0;
}
