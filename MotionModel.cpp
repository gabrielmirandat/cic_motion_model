#include <MotionModel.h>
#include <math.h>
#include <ctime>

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
    Eigen::Vector3f state(0.0,0.0,0.0);

    mm.sample_motion_model_odometry(u, state);

    return 0;
}
