#pragma once

//Kortex(kinova) API
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

//Useful function for controlling kinova Gen3
#include <ExtendedJointPosition.hpp>
#include "kinova_controller/robot/kinova_gen3_base.hpp"

// for base F/T sensor
#include "kinova_controller/sensors/ati_ft.hpp"

#define PORT 10000
#define PORT_REAL_TIME 10001

class KinovaGen3: public KinovaGen3Base
{
    public:
    KinovaGen3();
    ~KinovaGen3();

    virtual bool init(std::shared_ptr<RobotState> robot_state_arg) override;
    //destroy
    virtual void destroy() override;
    virtual void update_state() override;
    virtual bool set_joint_torque(const Eigen::VectorXd &u) override;
    virtual void add_render(const MCP_EP_Visualization & mcp_ep_visualization) override;
    virtual void apply_force(const MujocoApplyForce & mujoco_apply_force) override;

    private:
    bool move2home_position();
    std::function<void(Kinova::Api::Base::ActionNotification)>
    create_event_listener_by_promise(std::promise<Kinova::Api::Base::ActionEvent>& finish_promise);
    bool set_current_mode();
    Gen3State get_gen3_state(){return gen3_state;}
    void convert_joint_position(Eigen::Matrix<double, 7, 1>& joint_positions);


    // configuration
    std::string gen3_ip;
    std::shared_ptr<RobotState> robot_state;
    Gen3State gen3_state;
    std::array<float, 7> input_current_limit={10,10,10,10,6,6,6};

    //for base F/T sensor
    std::shared_ptr<ATI_FT> ft_sensor;
    std::string ft_sensor_ip;

    //filter
    double sampling_time;
    LPF lpf_dq;
    double lpf_dq_cutoff_freq;
    LPF lpf_tau_j;
    double lpf_tau_j_cutoff_freq;

    //for convert joint position
    std::array<int, 7> is_limit = {0,1,0,1,0,1,0};

    // Kortex API
    //CORE VARIABLES OF CONTROL GEN3
    Kinova::Api::Base::BaseClient *base;
    Kinova::Api::BaseCyclic::BaseCyclicClient *base_cyclic;
    Kinova::Api::ActuatorConfig::ActuatorConfigClient *actuator_config;
    Kinova::Api::SessionManager* session_manager;
    Kinova::Api::SessionManager* session_manager_real_time;
    Kinova::Api::RouterClient* router;
    Kinova::Api::TransportClientTcp* transport;
    Kinova::Api::RouterClient* router_real_time;
    Kinova::Api::TransportClientUdp* transport_real_time;
    Kinova::Api::BaseCyclic::Feedback base_feedback;
    Kinova::Api::BaseCyclic::Command  base_command;

};