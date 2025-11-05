#include "node/kinova_controller_node.hpp"

KinovaControllerNode::KinovaControllerNode(const rclcpp::NodeOptions & options)
: KinovaControllerNode("kinova_controller",  options)
{}

KinovaControllerNode::KinovaControllerNode(const std::string & node_name, const rclcpp::NodeOptions & options):
LifecycleNode(node_name, options),
state_topic_name(declare_parameter<std::string>("state_topic_name", "extended_robot_state")),
opt_output_fixed_topic_name(declare_parameter<std::string>("opt_output_fixed_topic_name", "opt_ctrl_output_fixed")),
opt_output_topic_name(declare_parameter<std::string>("opt_output_topic_name", "opt_ctrl_output")),
topic_stats_topic_name(declare_parameter<std::string>("topic_stats_topic_name", "topic_stats")),
topic_stats_publish_period(std::chrono::milliseconds{declare_parameter<std::uint16_t>("topic_stats_publish_period_ms", 1000U)}),
mujoco_visualize_topic_name(declare_parameter<std::string>("mujoco_visualize_topic_name", "mujoco_visualization")),
mujoco_apply_force_topic_name(declare_parameter<std::string>("mujoco_apply_force_topic_name", "mujoco_apply_force")),
controller_config_topic_name(declare_parameter<std::string>("controller_config_topic_name", "controller_config")),
desired_joint_value_topic_name(declare_parameter<std::string>("desired_joint_value_topic_name", "desired_joint_value")),
desired_task_value_topic_name(declare_parameter<std::string>("desired_task_value_topic_name", "desired_task_value")),
controller_logging_topic_name(declare_parameter<std::string>("controller_logging_topic_name", "controller_logging")),
deadline_duration{std::chrono::milliseconds{declare_parameter<std::uint16_t>("deadline_duration_ms", 0U)}},
update_gain_topic_name(declare_parameter<std::string>("update_gain_topic_name", "update_gain")),
main_control_loop_period(std::chrono::microseconds{declare_parameter<std::uint16_t>("state_publish_period_us", 1000U)}),
enable_topic_stats(declare_parameter<bool>("enable_topic_stats", false)),
initialize_ft_sensor_topic_name(declare_parameter<std::string>("initialize_ft_sensor_topic_name", "initialize_ft_sensor")),
num_missed_deadlines_pub{0U},
num_missed_deadlines_sub{0U}
{   
    opt_data = std::make_shared<OptData>();
    RCLCPP_INFO(get_logger(), "KinovaControllerNode constructor called");
    create_main_control_loop_callback();
    create_state_publisher();
    create_opt_output_subscriber();
    create_controller_config_subscriber();
    create_desired_joint_value_subscriber();
    create_desired_task_value_subscriber();
    create_update_gain_subscriber();
}

KinovaControllerNode::~KinovaControllerNode()
{
    RCLCPP_INFO(get_logger(), "KinovaControllerNode destructor called");
}

void KinovaControllerNode::main_control_loop()
{    
    // update robot_state from kinova gen3
    kinova_gen3->update_state();
    robot_state->time_now = this->now();


    if(robot_state->is_simulation)
    {
        // apply force
        MujocoApplyForce converted_mujoco_apply_force = robot_state->convert_force_frame(mujoco_apply_force);
        kinova_gen3->apply_force(converted_mujoco_apply_force);
    }
    
    // Get control input
    Eigen::VectorXd u = controller.get_control_input(controller_selector);

    // Error handling
    if(u.size() != robot_state->nu)
    {
        RCLCPP_WARN(get_logger(), "Control input size is not matched with robot state");
        RCLCPP_WARN(get_logger(), "Changed to gravity compensation mode");
        controller_selector = controller.select("GRAVITY_COMPENSATION");
        u = controller.get_control_input(controller_selector);
    }

    if(u.hasNaN())
    {
        RCLCPP_WARN(get_logger(), "Control input has NaN");
        RCLCPP_WARN(get_logger(), "Changed to gravity compensation mode");
        controller_selector = controller.select("GRAVITY_COMPENSATION");
        u = controller.get_control_input(controller_selector);
    }

    

    // Set control input
    kinova_gen3->set_joint_torque(u);
    // std::cout<<"main loop u: "<<u.transpose()<<std::endl;

    // fill state_message
    robot_state_msgs::msg::ExtendedState robot_state_msg;

    for (int i=0;i<robot_state->nq;i++)
    {
        robot_state_msg.q.push_back(robot_state->q(i));        
    }

    for(int i=0; i<robot_state->nv; i++)
    {   
        robot_state_msg.dq.push_back(robot_state->dq(i));
        // robot_state_msg.tau_ext.push_back(robot_state->tau_ext(i));
        robot_state_msg.tau_j.push_back(robot_state->tau_J(i));
        // robot_state_msg.effort.push_back(u(i));
    }

    // for energy tank
    if(controller_selector == CONTROLLER_SELECTOR::PASSIVE_CONTACT_FEEDBACK_MPC)
    {   
        const EnergyTankData & tank_data = controller.get_tank_data();
        const EnergyTankConfig & tank_config = controller.get_controller_gain().get_energy_tank_config(); 
        auto tank_state = & robot_state_msg.tank_state;

        tank_state->x_t = tank_data.xt;
        tank_state->energy = tank_data.Tt;
        tank_state->controller_power = tank_data.ut;
        tank_state->dissipated_power = tank_data.D;
        tank_state->dx_t = tank_data.dxt;
        tank_state->min_energy = tank_config.min_energy;
        tank_state->max_energy = tank_config.max_energy;
        
        // robot_state_msg.x_t = controller.get_tank().x_t;
        // robot_state_msg.energy = controller.get_tank().T_t;
    }
    else
    {   
        const EnergyTankConfig & tank_config = controller.get_controller_gain().get_energy_tank_config(); 
        const double energy = tank_config.initial_energy;
        auto tank_state = & robot_state_msg.tank_state;
        
        tank_state->x_t = sqrt(2.0 * energy);
        tank_state->energy = energy;
        tank_state->controller_power = 0.0;
        tank_state->dissipated_power = 0.0;
        tank_state->dx_t = 0.0;
        tank_state->min_energy = tank_config.min_energy;
        tank_state->max_energy = tank_config.max_energy;
    }

    // for base F/T sensor
    if(robot_state->base_ft)
    {
        for (int i=0; i<robot_state->base_wrench_ext.size(); i++)
        {
            robot_state_msg.base_wrench_ext.push_back(robot_state->base_wrench_ext(i));
        }
    }


    // end-effector pose
    auto ee_htm = robot_state->get_ee_pose("end_effector");
    for(int i=0; i<3; i++)
    {
        robot_state_msg.ee_pos[i] = (ee_htm.translation()(i));
    }
    // for(int i=0; i<9; i++)
    // {
    //     robot_state_msg.ee_rot[i] = (ee_htm.rotation()(i));
    // }

    // Publish state
    state_pub->publish(robot_state_msg);

    // publish controller logging
    if(robot_state->is_controller_logging)
    {
        publish_controller_logging();
    
    }
}

void KinovaControllerNode::create_real_robot()
{   
    kinova_gen3 = std::make_shared<KinovaGen3>();
}

void KinovaControllerNode::create_simulation_robot(mj::Simulate* sim)
{
    kinova_gen3 = std::make_shared<KinovaGen3Simulation>(sim);
}

void KinovaControllerNode::create_state_publisher()
{
    rclcpp::PublisherOptions sensor_publisher_options;
    sensor_publisher_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineOfferedInfo &) -> void
        {
            num_missed_deadlines_pub++;
        };
    state_pub = this->create_publisher<robot_state_msgs::msg::ExtendedState>(
        state_topic_name,
        rclcpp::QoS(10).deadline(deadline_duration),
        sensor_publisher_options
    );
}

void KinovaControllerNode::create_main_control_loop_callback()
{   
    // Callback function for state timer
    auto main_loop_callback = std::bind(&KinovaControllerNode::main_control_loop, this);
    main_control_loop_timer = this->create_wall_timer(main_control_loop_period, main_loop_callback);
    
    // cancel immediately to prevent triggering it in this state
    main_control_loop_timer->cancel();
}

void KinovaControllerNode::create_mujoco_visualize_subscriber()
{
    mcp_ep_mujoco_visualize_sub = this->create_subscription<mcp_ep_msgs::msg::MujocoVisualize>(
        mujoco_visualize_topic_name,
        rclcpp::QoS(10),
        std::bind(&KinovaControllerNode::mujoco_visualize_callback, this, std::placeholders::_1)
    );
}


void KinovaControllerNode::mujoco_visualize_callback(const mcp_ep_msgs::msg::MujocoVisualize::SharedPtr msg)
{
    int num_contacts = msg->num_contacts;
    mcp_ep_visualization.particle_pos.resize(num_contacts);
    mcp_ep_visualization.particle_force.resize(num_contacts); 
    mcp_ep_visualization.points.resize(num_contacts);
    mcp_ep_visualization.forces.resize(num_contacts);
    mcp_ep_visualization.exploit_particle_num.resize(num_contacts);

    for(int i=0; i<num_contacts; i++)
    {
        mcp_ep_visualization.points[i](0) = msg->visual_points_mujoco[i].x;
        mcp_ep_visualization.points[i](1) = msg->visual_points_mujoco[i].y;
        mcp_ep_visualization.points[i](2) = msg->visual_points_mujoco[i].z;

        mcp_ep_visualization.forces[i](0) = msg->visual_forces_mujoco[i].x;
        mcp_ep_visualization.forces[i](1) = msg->visual_forces_mujoco[i].y;
        mcp_ep_visualization.forces[i](2) = msg->visual_forces_mujoco[i].z;

        int particle_set_size = msg->visual_particle_pos_mujoco[i].vector_array.size();
        mcp_ep_visualization.particle_pos[i].resize(particle_set_size);
        mcp_ep_visualization.particle_force[i].resize(particle_set_size);
        
        for(int j=0; j<particle_set_size; j++)
        {
            mcp_ep_visualization.particle_pos[i][j](0) = msg->visual_particle_pos_mujoco[i].vector_array[j].x;
            mcp_ep_visualization.particle_pos[i][j](1) = msg->visual_particle_pos_mujoco[i].vector_array[j].y;
            mcp_ep_visualization.particle_pos[i][j](2) = msg->visual_particle_pos_mujoco[i].vector_array[j].z;

            mcp_ep_visualization.particle_force[i][j](0) = msg->visual_particle_force_mujoco[i].vector_array[j].x;
            mcp_ep_visualization.particle_force[i][j](1) = msg->visual_particle_force_mujoco[i].vector_array[j].y;
            mcp_ep_visualization.particle_force[i][j](2) = msg->visual_particle_force_mujoco[i].vector_array[j].z;
        }

        mcp_ep_visualization.exploit_particle_num[i] = msg->exploit_particle_num_mujoco[i];
    }
    mcp_ep_visualization.is_visualize = msg->is_visualize;

    kinova_gen3->add_render(mcp_ep_visualization);
}


void KinovaControllerNode::create_opt_output_fixed_subscriber()
{
    const size_t queue_size = 1;
    // Pre-allocates message in a pool
    using rclcpp::strategies::message_pool_memory_strategy::MessagePoolMemoryStrategy;
    auto command_msg_strategy = std::make_shared<MessagePoolMemoryStrategy<opt_ctrl_msgs::msg::OutputFixed, 1>>();
    rclcpp::SubscriptionOptions command_subscription_options;
    command_subscription_options.event_callbacks.deadline_callback =
        [this](rclcpp::QOSDeadlineRequestedInfo &) -> void
        {
            num_missed_deadlines_sub++;
        };

    if (enable_topic_stats) 
    {
        command_subscription_options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;
        command_subscription_options.topic_stats_options.publish_topic = topic_stats_topic_name;
        command_subscription_options.topic_stats_options.publish_period = topic_stats_publish_period;
    }

    opt_output_fixed_sub = this->create_subscription<opt_ctrl_msgs::msg::OutputFixed>(
        opt_output_fixed_topic_name,
        rclcpp::QoS(10).deadline(deadline_duration),
        std::bind(&KinovaControllerNode::opt_output_fixed_callback, this, std::placeholders::_1),
        command_subscription_options,
        command_msg_strategy);
}

void KinovaControllerNode::opt_output_fixed_callback(const opt_ctrl_msgs::msg::OutputFixed::SharedPtr msg)
{
    opt_data->K = Eigen::Map<Eigen::MatrixXd>(msg->cap_k0.data(), robot_state->nu, robot_state->ndx);
    opt_data->k = Eigen::Map<Eigen::VectorXd>(msg->k0.data(), msg->k0.size());
    opt_data->u0 = Eigen::Map<Eigen::VectorXd>(msg->u0.data(), msg->u0.size());
    opt_data->x0 = Eigen::Map<Eigen::VectorXd>(msg->x0.data(), msg->x0.size());
    opt_data->use_lpf = msg->use_lpf;
    opt_data->use_approximation = msg->use_approximation;
}

void KinovaControllerNode::create_opt_output_subscriber()
{
    opt_output_sub = this->create_subscription<opt_ctrl_msgs::msg::Output>(
        opt_output_topic_name,
        rclcpp::QoS(10),
        std::bind(&KinovaControllerNode::opt_output_callback, this, std::placeholders::_1)
    );
}

void KinovaControllerNode::opt_output_callback(const opt_ctrl_msgs::msg::Output::SharedPtr msg)
{
    opt_data->K = Eigen::Map<Eigen::MatrixXd>(msg->cap_k0.data(), robot_state->nu, robot_state->ndx);
    opt_data->k = Eigen::Map<Eigen::VectorXd>(msg->k0.data(), msg->k0.size());
    opt_data->u0 = Eigen::Map<Eigen::VectorXd>(msg->u0.data(), msg->u0.size());
    opt_data->x0 = Eigen::Map<Eigen::VectorXd>(msg->x0.data(), msg->x0.size());
    opt_data->use_lpf = msg->use_lpf;
    opt_data->use_approximation = msg->use_approximation;
    opt_data->t0 = msg->time;
}

void KinovaControllerNode::create_mujoco_apply_force_subscriber()
{
    mcp_ep_mujoco_apply_force_sub = this->create_subscription<mcp_ep_msgs::msg::MujocoApplyForce>(
        mujoco_apply_force_topic_name,
        rclcpp::QoS(10),
        std::bind(&KinovaControllerNode::mujoco_apply_force_callback, this, std::placeholders::_1)
    );
}
void KinovaControllerNode::mujoco_apply_force_callback(const mcp_ep_msgs::msg::MujocoApplyForce::SharedPtr msg)
{   
    int num_contacts = msg->num_contacts;
    mujoco_apply_force.resize(num_contacts);
    for(int i=0; i<num_contacts; i++)
    {
        mujoco_apply_force.contact_point[i](0) = msg->contact_point[i].x;
        mujoco_apply_force.contact_point[i](1) = msg->contact_point[i].y;
        mujoco_apply_force.contact_point[i](2) = msg->contact_point[i].z;

        mujoco_apply_force.contact_force[i](0) = msg->contact_force[i].x;
        mujoco_apply_force.contact_force[i](1) = msg->contact_force[i].y;
        mujoco_apply_force.contact_force[i](2) = msg->contact_force[i].z;

        mujoco_apply_force.contact_link_com[i](0) = msg->contact_link_com[i].x;
        mujoco_apply_force.contact_link_com[i](1) = msg->contact_link_com[i].y;
        mujoco_apply_force.contact_link_com[i](2) = msg->contact_link_com[i].z;

        mujoco_apply_force.contact_link_id[i] = msg->contact_link_index[i];
    }

    mujoco_apply_force.k_env = msg->k_env;

    // for environment
    if(mujoco_apply_force.k_env > 0.0)
    {   
        mujoco_apply_force.K_env.resize(num_contacts);
        mujoco_apply_force.env_location.resize(num_contacts);

        for(int i=0; i<num_contacts; i++)
        {   
            int contact_link_id = mujoco_apply_force.contact_link_id[i];
            auto se3 = robot_state->pinocchio_data.oMi[contact_link_id];

            //get contact frame orientation
            Eigen::Vector3d cf_x, cf_y, cf_z;
            Eigen::Matrix3d contact_frame_rot, R_wc, K_c;
            cf_z = mujoco_apply_force.contact_force[i]/mujoco_apply_force.contact_force[i].norm();
            cf_x = {0.1, 0.1, 0.1};
            if(cf_z.dot(cf_x) < 0.0001)
            {
                cf_x={0.3, 0.1, -0.3};
            }
            cf_x = cf_x - cf_z.dot(cf_x)*cf_z;
            cf_x = cf_x/cf_x.norm();

            cf_y = cf_z.cross(cf_x);
            
            //rotation joint frame to contact frame
            contact_frame_rot.col(0) = cf_x;
            contact_frame_rot.col(1) = cf_y;
            contact_frame_rot.col(2) = cf_z;
            
            //rotation world frame to contact frame
            R_wc = se3.rotation()*contact_frame_rot;

            //update spring stiffness
            // this->update_spring_stiffness(i);

            K_c.setZero();
            K_c(2,2) = mujoco_apply_force.k_env;

            mujoco_apply_force.K_env[i] = R_wc * K_c * R_wc.transpose();

            //x_env w.r.t contact frame
            Eigen::Vector3d x_env_c;
            x_env_c << 0, 0, mujoco_apply_force.contact_force[i].norm()/mujoco_apply_force.k_env;

            //x_env w.r.t joint frame
            Eigen::Vector3d x_env_j  = mujoco_apply_force.contact_point[i] + (contact_frame_rot*x_env_c);

            //x_env : w.r.t world frame
            mujoco_apply_force.env_location[i] = se3.translation() + se3.rotation()*x_env_j;
        }
    }

    mujoco_apply_force.is_visualize = msg->is_visualize;
}

void KinovaControllerNode::create_controller_config_subscriber()
{   
    auto callback = [this](const controller_interface_msgs::msg::ControllerConfig::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), "Received controller config message");
        std::string controller_type = msg->controller_selector;
        controller_selector = controller.select(controller_type);
    };

    controller_config_sub = this->create_subscription<controller_interface_msgs::msg::ControllerConfig>(
        controller_config_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

void KinovaControllerNode::create_desired_joint_value_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::SetDesiredJointValue::SharedPtr msg) -> void
    {
        Eigen::VectorXd q_d_temp = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->q_d.data(), msg->q_d.size());        
        robot_state->convert_q(q_d_temp);
        robot_state->q_d = q_d_temp;
        robot_state->dq_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->dq_d.data(), msg->dq_d.size());
        robot_state->ddq_d = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->ddq_d.data(), msg->ddq_d.size());
        robot_state->Set_trajectory = msg->set_traj;

    };

    desired_joint_value_sub = this->create_subscription<controller_interface_msgs::msg::SetDesiredJointValue>(
        desired_joint_value_topic_name,
        rclcpp::QoS(10),
        callback
    );

}

void KinovaControllerNode::create_desired_task_value_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::SetDesiredTaskValue::SharedPtr msg) -> void
    {
        Eigen::VectorXd quat = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->quat_d.data(), msg->quat_d.size());
        Eigen::Matrix3d rot;

        if(quat.size()!= 0)
        {
            rot = Eigen::Quaterniond(quat(0), quat(1), quat(2), quat(3)).toRotationMatrix(); // w, x, y, z
        }
        else
        {
            rot = Eigen::Map<Eigen::Matrix3d, Eigen::Unaligned>(msg->rot_d.data(), 3, 3);
        }
        
        Eigen::Vector3d pos = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(msg->pos_d.data(), msg->pos_d.size());

        Eigen::Matrix4d htm_d_mat; htm_d_mat.setIdentity();
        htm_d_mat.block(0,0,3,3) = rot;
        htm_d_mat.block(0,3,3,1) = pos;

        pinocchio::SE3 htm_d(htm_d_mat);

        robot_state->htm_d = htm_d;
        robot_state->Set_trajectory = msg->set_traj;
    };

    desired_task_value_sub = this->create_subscription<controller_interface_msgs::msg::SetDesiredTaskValue>(
        desired_task_value_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

void KinovaControllerNode::create_initialize_ft_sensor_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::InitializeFtSensor::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), "Received initialize ft sensor message");
        robot_state->init_base_ft = msg->init;
        robot_state->avg_filter_count = msg->avg_filter_count;
    };

    initialize_ft_sensor_sub = this->create_subscription<controller_interface_msgs::msg::InitializeFtSensor>(
        initialize_ft_sensor_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

void KinovaControllerNode::create_controller_logging_publisher()
{   
    controller_logging_pub = this->create_publisher<robot_state_msgs::msg::ControllerLogging>(
        controller_logging_topic_name,
        rclcpp::QoS(10)
    );

    controller_logging_pub->on_activate();
}
void KinovaControllerNode::publish_controller_logging()
{
    const ControllerLoggingData & controller_logging_data = robot_state->controller_logging_data;

    robot_state_msgs::msg::ControllerLogging msg;

    msg.time = this->now();
    for(int i=0; i<robot_state->nq; i++)
    {
        msg.nominal_theta.push_back(controller_logging_data.nominal_theta(i));
        msg.q_des.push_back(controller_logging_data.q_des(i));
        msg.theta_des.push_back(controller_logging_data.theta_des(i));
    }

    for(int i=0; i<robot_state->nv; i++)
    {
        msg.e_dn.push_back(controller_logging_data.e_dn(i));
        msg.e_nr.push_back(controller_logging_data.e_nr(i));
        // msg.edot_dn.push_back(controller_logging_data.edot_dn(i));
        // msg.edot_nr.push_back(controller_logging_data.edot_nr(i));
        // msg.nominal_ddtheta.push_back(controller_logging_data.nominal_ddtheta(i));
        msg.nominal_dtheta.push_back(controller_logging_data.nominal_dtheta(i));
        msg.tau_c.push_back(controller_logging_data.tau_c(i));
        // msg.tau_f.push_back(controller_logging_data.tau_f(i));
        msg.est_tau_f.push_back(controller_logging_data.est_tau_f(i));
        // msg.u.push_back(controller_logging_data.u(i));
        
    }

    controller_logging_pub->publish(msg);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_configure(const rclcpp_lifecycle::State &)
{   
    RCLCPP_INFO(get_logger(), "Configuring");

    // create controller logging publisher
    if(robot_state->is_controller_logging)
    {
        create_controller_logging_publisher();
    }

    // Initialize kinova gen3 & set current control mode
    if(!kinova_gen3->init(robot_state))
    {   
        RCLCPP_ERROR(get_logger(), "Failed to initialize kinova gen3");
        return LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Robot is initialized");
    }
    
    // for subscribing ft sensor initialization msg
    if(robot_state->base_ft && !robot_state->is_simulation)
    {
        create_initialize_ft_sensor_subscriber();
    }
    
    controller.assign_robot_state(robot_state);
    controller.assign_opt_data(opt_data);
    if(!robot_state->is_simulation)
    {
        // additional initialize process (3s)
        RCLCPP_INFO(get_logger(), "Get initial state...");
        rclcpp::Rate loop_rate(1000); unsigned int loop_counter = 0;
        while(rclcpp::ok() && loop_counter < 1500)
        {   
            // Get state
            kinova_gen3->update_state();

            // Get control input
            const Eigen::VectorXd u = controller.get_control_input(CONTROLLER_SELECTOR::GRAVITY_COMPENSATION);

            // Set control input
            kinova_gen3->set_joint_torque(u);
            loop_counter++;

            loop_rate.sleep();
        }

        RCLCPP_INFO(get_logger(), "Get initial state done!");
    }
    else
    {
        create_mujoco_visualize_subscriber();
        create_mujoco_apply_force_subscriber();
    }

    // get initial state
    kinova_gen3->update_state();

    // Initialize robot state & controller
    controller.init();
    std::string controller_type = robot_state->config["init_controller"].as<std::string>();
    controller_selector = controller.select(controller_type);

    //start main control loop
    state_pub->on_activate();
    main_control_loop_timer->reset();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void KinovaControllerNode::create_update_gain_subscriber()
{
    auto callback = [this](const controller_interface_msgs::msg::UpdateGain::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), "Received update gain message");
        if(msg->update)
        {   
            RCLCPP_INFO(get_logger(), "Update gain");
            robot_state->update_config();
            controller.update_controller_gain(robot_state->config);
        }
        
    };

    update_gain_sub = this->create_subscription<controller_interface_msgs::msg::UpdateGain>(
        update_gain_topic_name,
        rclcpp::QoS(10),
        callback
    );
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_activate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Activating");
    
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Deactivating");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Cleaning up");

    // Destroy kinova gen3: set position control mode
    kinova_gen3->destroy();
    state_pub->on_deactivate();
    main_control_loop_timer->cancel();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
KinovaControllerNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(get_logger(), "Shutting down");
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point,
// allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(KinovaControllerNode)