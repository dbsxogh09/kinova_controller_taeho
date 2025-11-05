#pragma once

#include "kinova_controller/data/energy_tank_data.hpp"
#include <iostream>
#include "kinova_controller/controller/controller_gain.hpp"

class EnergyTankBase
{   
  public:
    /// @brief calculate x_t, dx_t, u_t, y_t, T_t, u
    /// @param u: plant input 
    /// @param y: plant output
    EnergyTankBase();
    virtual void calc(EnergyTankData & energy_tank_data) = 0;
    virtual void calc_diff(EnergyTankData & energy_tank_data) = 0;

  protected:


     
    

};