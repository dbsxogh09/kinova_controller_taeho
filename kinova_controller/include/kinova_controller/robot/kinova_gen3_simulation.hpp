#pragma once

#include "kinova_controller/robot/kinova_gen3_base.hpp"

#include <mujoco/mujoco.h>
#include "glfw_dispatch.h"
#include "simulate.h"
#include "array_safety.h"
#include <sys/errno.h>
#include <unistd.h>

using namespace mujoco;

namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

using ::mujoco::Glfw;

class KinovaGen3Simulation: public KinovaGen3Base
{
    public:
    KinovaGen3Simulation(mj::Simulate* sim);
    ~KinovaGen3Simulation();

    virtual bool init(std::shared_ptr<RobotState> robot_state_arg) override;
    //destroy
    virtual void destroy() override;
    virtual void update_state() override;
    virtual bool set_joint_torque(const Eigen::VectorXd &u) override;
    virtual void add_render(const MCP_EP_Visualization & mcp_ep_visualization) override;
    virtual void apply_force(const MujocoApplyForce & mujoco_apply_force_arg) override;
    

    private:
    std::vector<int> joint_axis;
    MujocoApplyForce mujoco_apply_force;

    // simulate object encapsulates the UI
    mj::Simulate* sim;
    mjModel * LoadModel(const char* file, mj::Simulate& sim);

    std::shared_ptr<RobotState> robot_state;
    Gen3State gen3_state;
    // model and data
    mjModel* m = nullptr;
    mjData* d = nullptr;
    const int kErrorLength = 1024;

    // control noise variables
    mjtNum* ctrlnoise = nullptr;
};