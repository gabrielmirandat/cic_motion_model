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
    float delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);




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
