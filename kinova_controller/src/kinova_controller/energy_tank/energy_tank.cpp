#include "kinova_controller/energy_tank/energy_tank.hpp"

EnergyTank::EnergyTank()
{

}

void EnergyTank::init(const EnergyTankConfig & tank_config_arg)
{
    // set intial energy, T0 = 0.5 * x^2
    tank_config = tank_config_arg;
    
    T_max = tank_config.max_energy;
    T_min = tank_config.min_energy;
    Kd = tank_config.Kd;
}

void EnergyTank::calc(EnergyTankData & energy_tank_data)
{   
    // method #1: fix the alpha and beta as 1

    // update alpha and beta
    // alpha = 1;
    // beta = 1;
    
    // // update w
    // energy_tank_data.w = alpha / energy_tank_data.xt * energy_tank_data.controller_output;

    // //tank & controller connection
    // energy_tank_data.u = energy_tank_data.controller_output;
    // energy_tank_data.ut = - energy_tank_data.w.dot(energy_tank_data.y);

    // // calculate D (dissipated energy)
    // energy_tank_data.damping = Kd * energy_tank_data.y;
    // energy_tank_data.D = beta/energy_tank_data.xt * energy_tank_data.y.dot(energy_tank_data.damping);

    // // Tank dynamics: dx_t = D(x) + u_t, y_t = x_t
    // // calculate dx
    // energy_tank_data.dxt = energy_tank_data.D + energy_tank_data.ut;

    // // calculate y
    // energy_tank_data.yt = energy_tank_data.xt;

    // // calculate Tank energy
    // energy_tank_data.Tt = 0.5 * energy_tank_data.xt * energy_tank_data.xt;

    // method #2: alpha and beta are functions of Tt

    // update alpha and beta
    alpha = (energy_tank_data.Tt > T_min) ? 1 : 0;
    beta = (energy_tank_data.Tt < T_max) ? 1 : 0;
    

    energy_tank_data.alpha = alpha;
    energy_tank_data.beta = beta;
    
    // update w
    energy_tank_data.w = alpha / energy_tank_data.xt * energy_tank_data.controller_output;

    // update gamma
    // gamma = (energy_tank_data.w.dot(energy_tank_data.y) > 0) ? 1 : 0;
    // energy_tank_data.gamma = gamma;

    //tank & controller connection
    energy_tank_data.u = energy_tank_data.w * energy_tank_data.xt;
    energy_tank_data.ut = -  energy_tank_data.w.dot(energy_tank_data.y);
    // energy_tank_data.ut = - gamma * energy_tank_data.w.dot(energy_tank_data.y);

    // calculate D (dissipated energy)
    energy_tank_data.damping = Kd * energy_tank_data.y;
    energy_tank_data.D = beta/energy_tank_data.xt * energy_tank_data.y.dot(energy_tank_data.damping);

    // Tank dynamics: dx_t = D(x) + u_t, y_t = x_t
    // calculate dx
    energy_tank_data.dxt = energy_tank_data.D + energy_tank_data.ut;

    // calculate y
    energy_tank_data.yt = energy_tank_data.xt;

    // calculate Tank energy
    energy_tank_data.Tt = 0.5 * energy_tank_data.xt * energy_tank_data.xt;


}

void EnergyTank::calc_diff(EnergyTankData & energy_tank_data)
{   
    // calculate partial derivative
    // method #1: fix the alpha and beta as 1
    // energy_tank_data.dxt_q = Eigen::VectorXd::Zero(energy_tank_data.y.size());
    // energy_tank_data.dxt_xt = - beta/std::pow(energy_tank_data.xt,2) * energy_tank_data.y.transpose() * Kd * energy_tank_data.y 
    //                             + alpha/std::pow(energy_tank_data.xt,2) * energy_tank_data.controller_output.transpose() * energy_tank_data.y;
    // energy_tank_data.dxt_dq = 2* beta / energy_tank_data.xt * Kd * energy_tank_data.y - alpha / energy_tank_data.xt * energy_tank_data.controller_output;
    // energy_tank_data.dxt_u = -alpha/energy_tank_data.xt * energy_tank_data.y;

    // energy_tank_data.dTt_dq = energy_tank_data.xt * energy_tank_data.dxt_dq;

    // method #2: alpha and beta are functions of Tt
    alpha = energy_tank_data.alpha;
    beta = energy_tank_data.beta;
    // gamma = energy_tank_data.gamma;

    energy_tank_data.dxt_q = Eigen::VectorXd::Zero(energy_tank_data.y.size());
    energy_tank_data.dxt_xt = - beta/std::pow(energy_tank_data.xt,2) * energy_tank_data.y.transpose() * Kd * energy_tank_data.y 
                                + alpha/std::pow(energy_tank_data.xt,2) * energy_tank_data.controller_output.transpose() * energy_tank_data.y;
    // energy_tank_data.dxt_xt = - beta/std::pow(energy_tank_data.xt,2) * energy_tank_data.y.transpose() * Kd * energy_tank_data.y 
                                // + gamma * alpha/std::pow(energy_tank_data.xt,2) * energy_tank_data.controller_output.transpose() * energy_tank_data.y;

    energy_tank_data.dxt_dq = 2* beta / energy_tank_data.xt * Kd * energy_tank_data.y - alpha / energy_tank_data.xt * energy_tank_data.controller_output;
    energy_tank_data.dxt_u = -alpha/energy_tank_data.xt * energy_tank_data.y;
    // energy_tank_data.dxt_u = -gamma * alpha/energy_tank_data.xt * energy_tank_data.y;

}
