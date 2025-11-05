#pragma once

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "kinova_controller/data/robot_state.hpp"
#include "kinova_controller/data/nominal_plant.hpp"
#include "kinova_controller/data/coulomb_observer_state.hpp"
#include "kinova_controller/data/energy_tank_data.hpp"
#include "kinova_controller/energy_tank/energy_tank.hpp"
#include <unsupported/Eigen/MatrixFunctions>

#include <yaml-cpp/yaml.h>
#include "kinova_controller/data/controller_state.hpp"
#include "kinova_controller/data/opt_data.hpp"
#include "kinova_controller/controller/controller_gain.hpp"
#include "kinova_controller/filter/lpf.hpp"

#include "controller_interface_msgs/msg/controller_config.hpp"

#include "kinova_controller/math_type_define.h"

class Controller
{
    public:
    int loop_index_=0;
    Controller();

    void assign_robot_state(std::shared_ptr<RobotState> robot_state_arg);
    void assign_opt_data(std::shared_ptr<OptData> opt_data_arg);
    void init();
    CONTROLLER_SELECTOR select(const std::string & controller_type);
    Eigen::VectorXd get_control_input(const int& selector);
    EnergyTankData get_tank_data(){return energy_tank_data;}
    EnergyTank get_tank(){return energy_tank;}
    ControllerGain get_controller_gain(){return controller_gain;}
    void update_controller_gain(const YAML::Node& node);


    private:
    std::shared_ptr<RobotState> robot_state;
    std::shared_ptr<OptData> opt_data;
    ControllerGain controller_gain;
    int nq, nv, nu;
    double dt;
    CONTROLLER_SELECTOR selector_prev;

    double t;
    double frequency;
    double angular_velocity;
    double radius;
    double angle;

    //for lugre friction
    Eigen::VectorXd z_;

    Eigen::VectorXd sigma_hat_;

    //for NRIC
    NominalPlant nominal_plant;

    //for coulomb observer
    CoulombObserverState coulomb_observer_state;

    //for energy tank
    EnergyTankData energy_tank_data;
    EnergyTank energy_tank;

    //for flexible joint robot parameter
    Eigen::MatrixXd joint_stiffness_matrix_;
    Eigen::MatrixXd rotor_inertia_matrix_;

    //for low-pass filtering optimal control input
    LPF lpf_opt_u;

    
    //*****************************Controller**********************************//

    bool init_PD_joint_controller();
    Eigen::VectorXd PD_joint_controller();

    bool init_PD_task_controller();
    Eigen::VectorXd PD_task_controller();

    bool init_NRIC_joint_controller();
    Eigen::VectorXd NRIC_joint_controller();

    bool init_NRIC_task_controller();
    Eigen::VectorXd NRIC_task_controller();

    bool init_FRIC_joint_controller();
    Eigen::VectorXd FRIC_joint_controller();

    bool init_FRIC_task_controller();
    Eigen::VectorXd FRIC_task_controller();

    bool init_gravity_compensation();
    Eigen::VectorXd gravity_compensation();

    bool init_contact_feedback_mpc();
    Eigen::VectorXd contact_feedback_mpc();

    bool init_passive_contact_feedback_mpc();
    Eigen::VectorXd passive_contact_feedback_mpc();

    //torque control interface
    bool init_first_order_friction_compensation();
    Eigen::VectorXd first_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_second_order_friction_compensation();
    Eigen::VectorXd second_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_third_order_friction_compensation();
    Eigen::VectorXd third_order_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_intertia_reshaping();
    Eigen::VectorXd inertia_reshaping(const Eigen::VectorXd &desired_u);
    bool init_l1_friction_compensation();
    Eigen::VectorXd l1_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_implicit_l1_friction_compensation();
    Eigen::VectorXd implicit_l1_friction_compensation(const Eigen::VectorXd &desired_u);
    bool init_coulomb_observer_friction_compensation();
    Eigen::VectorXd coulomb_observer_friction_compensation(const Eigen::VectorXd &desired_u);

    //*************************************************************************//
};