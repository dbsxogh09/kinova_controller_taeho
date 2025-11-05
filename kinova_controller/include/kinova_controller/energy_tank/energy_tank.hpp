#pragma once

#include "kinova_controller/energy_tank/energy_tank_base.hpp"
#include <iostream>

class EnergyTank: public EnergyTankBase
{   
  public:
    /// @brief calculate x_t, dx_t, u_t, y_t, T_t, u
    /// @param u: plant input 
    /// @param y: plant output
    EnergyTank();
    void init(const EnergyTankConfig & config_arg);
    virtual void calc(EnergyTankData & energy_tank_data_arg) override;
    virtual void calc_diff(EnergyTankData & energy_tank_data_arg) override;
    Eigen::MatrixXd get_Kd() {return Kd;}

  private:
    // related to energy tank configuration
    EnergyTankConfig tank_config;
    
    double T_max;
    double T_min;
    Eigen::MatrixXd Kd; // dissipation gain
    int alpha; // 1 if T > T_min, 0 otherwise
    int beta; // 1 if T < T_max, 0 otherwise
    int gamma; // 1 if w*y >0, 0 otherwise

};