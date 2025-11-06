#pragma once

#include <Eigen/Dense>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"

#include <yaml-cpp/yaml.h>
#include "kinova_controller/data/gen3_state.hpp"
#include "kinova_controller/data/mujoco_apply_force.hpp"
#include <rclcpp/rclcpp.hpp>
#include "kinova_controller/data/controller_logging_data.hpp"

class RobotState
{   
    public:
    RobotState(const YAML::Node &config_arg, const pinocchio::Model &pinocchio_model_arg, const std::string & config_path_arg);

    void update_robot_state(const Gen3State & gen3_state);
    void update_base_wrench(const Eigen::VectorXd& base_wrench_measured_arg);
    void update_DOB();

    Eigen::VectorXd integrate(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const double &dt);
    Eigen::VectorXd difference(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2); // dqout = q2 - q1

    Eigen::MatrixXd get_mass(const Eigen::VectorXd &q);
    Eigen::MatrixXd get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq);
    Eigen::MatrixXd get_gravity(const Eigen::VectorXd &q);
    
    Eigen::MatrixXd get_ee_jacobian_w(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::MatrixXd get_ee_jacobian_w(const std::string & ee_name);
    
    Eigen::MatrixXd get_ee_jacobian_b(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::MatrixXd get_ee_jacobian_b(const std::string & ee_name);

    Eigen::Affine3d get_ee_pose(const Eigen::VectorXd &q, const std::string & ee_name);
    Eigen::Affine3d get_ee_pose(const std::string & ee_name);

    Eigen::VectorXd get_base_ext_wrench(const Eigen::VectorXd &gacc);
    Eigen::VectorXd get_base_nominal_wrench(const Eigen::VectorXd &gacc);

    Eigen::VectorXd get_joint_nominal_torque(const Eigen::VectorXd &gacc);
    
    MujocoApplyForce convert_force_frame(const MujocoApplyForce & mujoco_apply_force);


    YAML::Node get_config() { return config; }
    void update_config();
    void convert_q(Gen3State &gen3_state);
    void convert_q(Eigen::VectorXd &q_arg);

    pinocchio::Model pinocchio_model;
    pinocchio::Data pinocchio_data;
    YAML::Node config;
    const std::string config_path; 

    Eigen::VectorXd q; // link position
    Eigen::VectorXd dq; // qdot

    Eigen::VectorXd q_d; // desired link positions
    Eigen::VectorXd dq_d; // desired link velocity

    Eigen::VectorXd ddq_d; // desired link acceleration

    Eigen::VectorXd theta; // motor position
    Eigen::VectorXd dtheta;

    Eigen::VectorXd theta_next;
    Eigen::VectorXd x_des_prev;

    Eigen::VectorXd des_vel_prev;

    Eigen::VectorXd tau_J; // JTS measurement
    Eigen::VectorXd base_wrench_measured; // base F/T sensor measurement
    Eigen::VectorXd joint_temperatures; // joint motor temperatures

    Eigen::VectorXd tau_ext; // estimated external joint torque
    Eigen::VectorXd base_wrench_ext; // estimated external joint torque

    std::vector<Eigen::Affine3d> ee_htm; // End-effector position
    std::vector<Eigen::Affine3d> ee_htm_d;

    pinocchio::SE3 htm_d; // desried end-effector pose

    rclcpp::Time time_now;
    double time = 0.0;
    
    Eigen::VectorXd theta_init;

    bool is_rigid;
    bool is_simulation;

    bool Set_trajectory = false;

    const int nq;
    const int nv;
    const int nu;
    const int nx;
    const int ndx;
    double dt;

    bool base_ft;
    bool init_base_ft = true;

    Eigen::VectorXd bias;
    bool init_tau_j = true;

    unsigned int count = 1;
    int avg_filter_count = 1000;

    MujocoApplyForce converted_mujoco_apply_force; // for force transformation

    //for data logging
    bool is_controller_logging;
    ControllerLoggingData controller_logging_data;

    void print()
    { 
        std::cout<<"----------------------------------------"<<std::endl;
        std::cout<<"q : " << q.transpose()<<std::endl;
        std::cout<<"dq : " << dq.transpose()<<std::endl;
        std::cout<<"theta : " << theta.transpose()<<std::endl;
        std::cout<<"dtheta : " << dtheta.transpose()<<std::endl;
        std::cout<<"q_d : " << q_d.transpose()<<std::endl;
        std::cout<<"dq_d : " << dq_d.transpose()<<std::endl;

    }

    private:
    Eigen::VectorXd save_integral_;

    // F/T sensor frame
    std::vector<std::string> base_ft_frame_names;
    std::vector<int> base_ft_frame_ids;

    // Base link frame
    std::vector<std::string> base_link_names;
    std::vector<int> base_link_ids;
    
    // Joint frame attached to the base link
    std::vector<std::string> base_joint_names;
    std::vector<int> base_joint_ids;

    Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d skew;
        skew << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(0), 0;
        return skew;
    }

};

