#pragma once

#include <Eigen/Dense>
#include "kinova_controller/controller/controller_gain.hpp"

struct EnergyTankData
{
    // related to energy tank
    double xt; // state
    double dxt; // time derivative of x
    double ut; // input
    double yt; // output
    double Tt; // energy
    Eigen::VectorXd w;

    int alpha;
    int beta;
    int gamma;

    // partial derivative for robot model
    Eigen::VectorXd dxt_q; 
    Eigen::VectorXd dxt_xt;
    Eigen::VectorXd dxt_dq;
    Eigen::VectorXd dxt_u;
    
    // related to plant input output
    Eigen::VectorXd u;
    Eigen::VectorXd y;
    Eigen::VectorXd damping;
    double D; // dissipated energy
    Eigen::VectorXd controller_output;

    EnergyTankData();
    void update_data(const double & x_t0, const Eigen::VectorXd & controller_output0, const Eigen::VectorXd & y0);
    void init(const EnergyTankConfig & config);
};