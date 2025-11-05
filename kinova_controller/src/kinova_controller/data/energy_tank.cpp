#include "kinova_controller/data/energy_tank.hpp"

void EnergyTank::init(const EnergyTankConfig & config)
{   
    // set intial energy, T0 = 0.5 * x^2
    T_t = config.initial_energy;
    T_max = config.max_energy;
    T_min = config.min_energy;
    Kd = config.Kd;

    x_t = sqrt(2.0 * T_t);
    dx_t = 0.0;
    u_t = 0.0;
    y_t = x_t;
    alpha = 1;
    beta = 1;
    
}

void EnergyTank::calc(const Eigen::VectorXd & controller_output, const Eigen::VectorXd & y_arg)
{   
    // update y (output of the plant: dq)
    y = y_arg;

    // update alpha and beta
    alpha = (T_t > T_min) ? 1 : 0;
    beta = (T_t < T_max) ? 1 : 0;

    // update w
    w = alpha / x_t * controller_output;

    //tank & controller connection
    u = w * y_t;
    u_t = - w.dot(y);

    // calculate D (dissipated energy)
    D = beta/x_t * y.dot(Kd * y);

    // Tank dynamics: dx_t = beta/x_t D(x) + u_t, y_t = x_t
    // calculate dx
    dx_t = D + u_t;

    // calculate x (integration)
    x_t= x_t + dx_t * dt;

    // calculate y
    y_t = x_t;

    // calculate Tank energy
    T_t = 0.5 * x_t * x_t;
}