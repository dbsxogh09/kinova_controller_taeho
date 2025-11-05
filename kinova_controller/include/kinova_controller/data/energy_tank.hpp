#pragma once

#include "kinova_controller/controller/controller_gain.hpp"

struct EnergyTank
{   
    // related to energy tank
    double x_t; // state
    double dx_t; // time derivative of x
    double u_t; // input
    double y_t; // output
    double T_t; // energy

    // related to energy tank configuration
    double T_max;
    double T_min;
    int alpha; // 1 if T > T_min, 0 otherwise
    int beta; // 1 if T < T_max, 0 otherwise
    const double dt = 0.001;
    Eigen::VectorXd w;

    // related to plant input output
    Eigen::VectorXd u;
    Eigen::VectorXd y;
    double D; // dissipated energy
    Eigen::MatrixXd Kd; // dissipation gain

    void init(const EnergyTankConfig & config);

    /// @brief calculate x_t, dx_t, u_t, y_t, T_t, u
    /// @param u: plant input 
    /// @param y: plant output
    void calc(const Eigen::VectorXd & controller_output, const Eigen::VectorXd & y_arg);

};