#include <MotionModel.h>
#include <math.h>
#include <ctime>

MotionModel::MotionModel()
{
    alpha1_ = 0.01; // translational error
    alpha2_ = 0.01; // translational error
    alpha3_ = 0.01; // angular error
    alpha4_ = 0.01; // angular error
    
    // seed e gerador usados nas funções de probabilidade do c++11
    seed_ = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed_);
    generator_ = generator;
}

// corrige o angulo, trazendo de volta p faixa [-pi, pi)
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
    // cout << sample(1.0) << endl;
    // cout << sample(0.5) << endl;
    // cout << sample(0.02) << endl;

    double delta_rot_1 = atan2(u.getXt(1) - u.getXtm1(1),u.getXt(0) - u.getXtm1(0)) - u.getXtm1(2);
    double delta_trans = sqrt((u.getXtm1(0) - u.getXt(0))*(u.getXtm1(0) - u.getXt(0))
                           + (u.getXtm1(1) - u.getXt(1))*(u.getXtm1(1) - u.getXt(1)));
    double delta_rot_2 = u.getXt(2) - u.getXtm1(2) - delta_rot_1;
    
    // traz os angulos para a faixa [-pi, pi) radianos, caso estejam fora
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
    // esse sample não tá dispersando as particulas, nao sei pq
    /*
    assert(b>0.0);
    srand(time(NULL));
    double sample_value = 0.0;
    for(int i=0; i < 12; i++)
    {
      double f = (double)rand() / RAND_MAX;
      sample_value += (-1)*b + f * (b - (-1)*b);
    }
    return sample_value/2;
    */
    // faz o sample com funções do c++11
    normal_distribution<double> errorDist(0.0, b);
    return errorDist(generator_);
}

int main()
{
    MotionModel mm;
    Control u;

    Eigen::Vector3d state, new_state;
    Eigen::VectorXd control(6);
    
    // numero de particulas
    int M = 1000;
    
    // conjuntos de particulas representado os estados passado e atual
    vector<Eigen::Vector3d> psXtm1 (M);
    vector<Eigen::Vector3d> psXt (M);
    
    // gnuplot
    Gnuplot gp;
    // vetores de dados que o gnuplot reconhece
    std::vector<double> filter_data_x (M),
        filter_data_y (M);
    // configura o plot
    gp << "set xrange [-50:50]\n"
    "set yrange [-50:50]\n"
    "set style fill transparent solid 0.1 noborder\n"
    "set style circle radius 0.1\n"
    "set title 'Location'\n";
    // faz o plot
    gp << "plot '-' with circles lc rgb 'black'\n";
    gp.send1d(boost::make_tuple(filter_data_x, filter_data_y));
    // espera 200ms (no ROS isso ja acontece)
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    // escolhi arbitrariamente 30 iteracoes
    for (int iter = 1; iter < 31; ++iter)
    {
        
        // essa parte gera uma sequencia de odometria de acordo c a figura 5.10 do livro
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
        
        // constroi o novo conjunto de particulas
        for (int m = 0; m < M; ++m) {
            // particula antiga
            state = psXtm1[m];
            // sorteia nova particula de acordo c o algoritmo que fizemos
            Eigen::Vector3d new_state = mm.sample_motion_model_odometry(u, state);
            // guarda a nova particula
            psXt[m] = new_state;
            // atualiza os dados para o gnuplot
            filter_data_x[m] = new_state[0];
            filter_data_y[m] = new_state[1];
        }
        
        // plota de novo os dados com o gnuplot e espera 200ms
        gp << "plot '-' with circles lc rgb 'black'\n";
        gp.send1d(boost::make_tuple(filter_data_x, filter_data_y));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        
        // agora o estado atual vai ser o novo estado passado
        psXtm1 = psXt;
        
    }

    return 0;
}
