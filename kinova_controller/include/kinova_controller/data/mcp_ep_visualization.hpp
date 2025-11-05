#pragma once

#include <Eigen/Dense>
#include <vector>
#include <iostream>

struct MCP_EP_Visualization
{
    //for mujoco visualization
    std::vector<std::vector<Eigen::Vector3d>> particle_pos;
    std::vector<std::vector<Eigen::Vector3d>> particle_force;

    //For visualize estimated contact point and force
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> forces;

    std::vector<int> exploit_particle_num;
    bool is_visualize;

    void print();
    
};