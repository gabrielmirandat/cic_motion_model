#include <MotionModel.h>
#include <math.h>

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
void MotionModel::sample_motion_model_odometry(Control u, Eigen::Vector3d state)
{
    u.print();
    double delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);
    double delta_trans = sqrt((u.getXtm1[0] - u.getXt[0])*(u.getXtm1[0] - u.getXt[0])
                           + (u.getXtm1[1] - u.getXt[1])*(u.getXtm1[1] - u.getXt[1]));
    double delta_rot_2 = u.getXt(2) - u.getXtm1(2) - delta_rot_1;


    double delta_rot_1_cor = delta_rot_1 - sample(alpha1_*fabs(delta_rot_1) + alpha2_*delta_trans);
    double delta_trans_cor = delta_trans - sample(alpha3_*delta_trans + alpha4_*(fabs(delta_rot_1) + fabs(delta_rot_2)));
    double delta_rot_2_cor = delta_rot_2 - sample(alpha1_*fabs(delta_rot_2) + alpha2_*delta_trans);

    return new_state;


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
