#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

struct MujocoApplyForce
{
    std::vector<Eigen::Vector3d> contact_point, contact_force, contact_link_com;
    std::vector<int> contact_link_id;
    bool is_visualize;

    double k_env; // vector
    std::vector<Eigen::Matrix3d> K_env; // matrix
    std::vector<Eigen::Vector3d> env_location;


    void clear();
    void resize(const int &size);
    void print();
};