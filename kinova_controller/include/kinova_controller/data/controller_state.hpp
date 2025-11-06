#pragma once

#include <Eigen/Dense>

enum CONTROLLER_SELECTOR
{
  JOINT_PD,
  JOINT_PD_FRIC,
  JOINT_NRIC,
  TASK_PD,
  TASK_PD_FRIC,
  TASK_NRIC,
  GRAVITY_COMPENSATION,
  CONTACT_FEEDBACK_MPC,
  PASSIVE_CONTACT_FEEDBACK_MPC,
};

enum TORQUE_INTERFACE
{
  FIRST_ORDER_FRIC,
  SECOND_ORDER_FRIC,
  THIRD_ORDER_FRIC,
  INERTIA_RESHAPING,
  L1_FRIC,
  IMPLICIT_L1_FRIC,
  COULOMB_OBSERVER,
};

// u = Kp * e + Kd * de
struct JOINT_PD_Gains
{
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Kd; // Derivative gain
};

struct JOINT_NRIC_Gains
{
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;
  Eigen::MatrixXd K;
  Eigen::MatrixXd gamma;
  Eigen::MatrixXd reflected_inertia;
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Ki; // Integral gain
};

struct TASK_PD_Gains
{
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Kd; // Derivative gain
};

struct TASK_NIRC_Gains
{
  Eigen::MatrixXd k1;
  Eigen::MatrixXd k2;
  Eigen::MatrixXd K;
  Eigen::MatrixXd gamma;
  Eigen::MatrixXd reflected_inertia;
  Eigen::MatrixXd Kp; // Proportional gain
  Eigen::MatrixXd Ki; // Integral gain
};

struct EnergyTankConfig
{
  double initial_energy;
  double max_energy;
  double min_energy;
  double mpc_run_out_time;
  Eigen::MatrixXd Kd;
};

struct JOINT_FRIC_Gains
{
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kd;
  Eigen::MatrixXd joint_stiffness_matrix;
  TORQUE_INTERFACE torque_interface;
};

struct TASK_FRIC_Gains
{
  Eigen::MatrixXd Kp;
  Eigen::MatrixXd Kd;
  Eigen::MatrixXd joint_stiffness_matrix;
  TORQUE_INTERFACE torque_interface;
};

//for torque control interface
struct FIRST_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
};

struct SECOND_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
  Eigen::MatrixXd Lp;
};

struct THIRD_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd L; 
  Eigen::MatrixXd Lp;
  Eigen::MatrixXd Li;
};

struct INERTIA_RESHAPING_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd desired_motor_inertia_matrix;
};

struct L1_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd As; 
  Eigen::MatrixXd Gamma;
  Eigen::MatrixXd W;
};

struct IMPLICIT_L1_FRIC_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd Gamma;
  Eigen::MatrixXd Gamma_p;
  Eigen::MatrixXd W;
};

struct COULOMB_OBSERVER_Gains
{
  Eigen::MatrixXd motor_inertia_matrix;
  Eigen::MatrixXd K;
  Eigen::MatrixXd L;
};