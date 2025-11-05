#pragma once

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

struct OptData
{
    Eigen::MatrixXd K;
    Eigen::VectorXd k;
    Eigen::VectorXd u0;
    Eigen::VectorXd x0;
    bool use_lpf;
    bool use_approximation;
    rclcpp::Time t0;
};