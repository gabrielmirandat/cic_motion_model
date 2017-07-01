#include <MotionModel.h>
#include <math.h>
#include <ctime>

MotionModel::MotionModel()
{
    alpha1_ = 0.01;
    alpha2_ = 0.01;
    alpha3_ = 0.01;
    alpha4_ = 0.01;
    
    seed_ = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed_);
    generator_ = generator;
    
}

double truncAngle (double theta) {
    if (theta >= M_PI)
        return (theta - 2.0*M_PI);
    else if (theta < -M_PI)
        return (theta + 2*M_PI);
    else return theta;
}

// in
// control u_t
// pose x_tm1
Eigen::Vector3d MotionModel::sample_motion_model_odometry(Control u, Eigen::Vector3d state)
{
    
    double delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);
    double delta_trans = sqrt((u.getXtm1(0) - u.getXt(0))*(u.getXtm1(0) - u.getXt(0))
                           + (u.getXtm1(1) - u.getXt(1))*(u.getXtm1(1) - u.getXt(1)));
    double delta_rot_2 = u.getXt(2) - u.getXtm1(2) - delta_rot_1;
    
    delta_rot_1 = truncAngle (delta_rot_1);
    delta_rot_2 = truncAngle (delta_rot_2);

    double delta_rot_1_corr = delta_rot_1 - sample(alpha1_*fabs(delta_rot_1) + alpha2_*delta_trans);
    double delta_trans_corr = delta_trans - sample(alpha3_*delta_trans + alpha4_*(fabs(delta_rot_1) + fabs(delta_rot_2)));
    double delta_rot_2_corr = delta_rot_2 - sample(alpha1_*fabs(delta_rot_2) + alpha2_*delta_trans);

    Eigen::Vector3d new_state;
    new_state[0] = state[0] + delta_trans_corr * cos (state[2] + delta_rot_1_corr);
    new_state[1] = state[1] + delta_trans_corr * sin (state[2] + delta_rot_1_corr);
    new_state[2] = truncAngle (state[2] + delta_rot_1_corr + delta_rot_2_corr);
    
    return new_state;
}

double MotionModel::sample(double b)
{
    /*
    srand(time(NULL));
    double sample_value = 0 ;
    for(int i = 0; i < 12; ++i)
        sample_value += (-1) + (static_cast<double>(rand()) / RAND_MAX) * (1 - (-1));

    return sample_value/b;
    */
    normal_distribution<double> errorDist(0.0, b);
    return errorDist(generator_);
}

int main()
{
    // u.print();
    // u.set(3,2.0);
    // u.print();

    MotionModel mm;
    Control u;
    Eigen::Vector3d state(0.0,0.0,0.0);
    Eigen::VectorXd control(6);
    
    int M = 1000;
    
    vector<Eigen::Vector3d> psXtm1 (M);
    vector<Eigen::Vector3d> psXt (M);
    
    Gnuplot gp;
    std::vector<double> filter_data_x (M),
        filter_data_y (M);
    gp << "set xrange [-50:50]\n"
    "set yrange [-50:50]\n"
    "set style fill transparent solid 0.1 noborder\n"
    "set style circle radius 0.1\n"
    "set title 'Location'\n";
    gp << "plot '-' with circles lc rgb 'black'\n";
    gp.send1d(boost::make_tuple(filter_data_x, filter_data_y));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    for (int iter = 1; iter < 31; ++iter) {
        
        Eigen::Vector3d new_control, old_control;
        if (iter < 11) {
            old_control = Eigen::Vector3d(2.0*(iter-1),0.,0.);
            if (iter < 10) new_control = Eigen::Vector3d(2.0*iter,0.,0.);
            else new_control = Eigen::Vector3d(20.0,0.,M_PI/2.0);
        } else if (iter < 21) {
            old_control = Eigen::Vector3d(20.0,2.0*(iter-11),M_PI/2.0);
            if (iter < 20) new_control = Eigen::Vector3d(20.0,2.0*(iter-10),M_PI/2.0);
            else new_control = Eigen::Vector3d(20.0,20.0,-M_PI);
        } else if (iter < 31) {
            old_control = Eigen::Vector3d(20.0-2.0*(iter-21),20.0,-M_PI);
            new_control = Eigen::Vector3d(20.0-2.0*(iter-20),20.0,-M_PI);
        }
        
        control << old_control, new_control;
        u.set(control);
        u.print();
        for (int m = 0; m < M; ++m) {
            state = psXtm1[m];
            Eigen::Vector3d new_state = mm.sample_motion_model_odometry(u, state);
            // cout << new_state[0] << "," << new_state[1] << "," << new_state[2] << "\n";
            psXt[m] = new_state;
            filter_data_x[m] = new_state[0];
            filter_data_y[m] = new_state[1];
        }
        
        gp << "plot '-' with circles lc rgb 'black'\n";
        gp.send1d(boost::make_tuple(filter_data_x, filter_data_y));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        psXtm1 = psXt;
        
    }
    
    return 0;
}
