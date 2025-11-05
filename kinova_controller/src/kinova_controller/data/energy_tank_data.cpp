#include "kinova_controller/data/energy_tank_data.hpp"

EnergyTankData::EnergyTankData()
{
    //do nothing
}

void EnergyTankData::update_data(const double & x_t0, const Eigen::VectorXd & controller_output0, const Eigen::VectorXd & y0)
{
    xt = x_t0;
    controller_output = controller_output0;
    y = y0;
}

void EnergyTankData::init(const EnergyTankConfig & config)
{   
    // set intial energy, T0 = 0.5 * x^2
    Tt = config.initial_energy;
    // T_max = config.max_energy;
    // T_min = config.min_energy;
    // Kd = config.Kd;

    xt = sqrt(2.0 * Tt);
    dxt = 0.0;
    ut = 0.0;
    yt = xt;
    alpha = 1;
    beta = 1;
    
}