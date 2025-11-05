#pragma once

#include <Eigen/Dense>

struct ControllerLoggingData
{
    Eigen::VectorXd tau_c;
    Eigen::VectorXd est_tau_f;
    Eigen::VectorXd tau_f;
    Eigen::VectorXd nominal_theta;
    Eigen::VectorXd nominal_dtheta;
    Eigen::VectorXd nominal_ddtheta;
    Eigen::VectorXd e_dn;
    Eigen::VectorXd edot_dn;
    Eigen::VectorXd e_nr;
    Eigen::VectorXd edot_nr;
    Eigen::VectorXd u;
    Eigen::VectorXd q_des;
    Eigen::VectorXd theta_des;

    void resize(const int & nq, const int & nv);
    void setZero();
};