#include "kinova_controller/data/robot_state.hpp"

RobotState::RobotState(const YAML::Node &config_arg, const pinocchio::Model &pinocchio_model_arg, const std::string & config_path_arg):
pinocchio_model(pinocchio_model_arg), pinocchio_data(pinocchio_model_arg), 
nq(pinocchio_model_arg.nq),
nv(pinocchio_model_arg.nv),
nu(pinocchio_model_arg.nv),
nx(pinocchio_model_arg.nv + pinocchio_model_arg.nq),
ndx(pinocchio_model_arg.nv *2),
config(config_arg),
config_path(config_path_arg)
{
    q = Eigen::VectorXd::Zero(nq);
    dq = Eigen::VectorXd::Zero(nv);
    q_d = Eigen::VectorXd::Zero(nq);
    dq_d = Eigen::VectorXd::Zero(nv);
    ddq_d = Eigen::VectorXd::Zero(nv);
    theta = Eigen::VectorXd::Zero(nv);
    dtheta = Eigen::VectorXd::Zero(nv);
    tau_J = Eigen::VectorXd::Zero(nv);
    joint_temperatures = Eigen::VectorXd::Zero(nv);
    base_wrench_measured = Eigen::VectorXd::Zero(6);
    tau_ext = Eigen::VectorXd::Zero(nv);
    base_wrench_ext = Eigen::VectorXd::Zero(6);
    ee_htm.resize(2);
    ee_htm_d.resize(2);
    time = 0.0;
    
    try
    {
        is_controller_logging = config["is_controller_logging"].as<bool>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: is_controller_logging is poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }

    try
    {   
        is_rigid = config["is_rigid"].as<bool>();
        base_ft = config["base_ft"].as<bool>();
        dt = config["dt"].as<double>();
        is_simulation = config["is_simulation"].as<bool>();
    }
    catch(YAML::Exception &e)
    {   
        std::cout<<"ERROR: is_rigid, base_ft, dt, is_simulation are poorly defined in the yaml file"<<std::endl;
        std::cout<<"YAML Exception : "<<e.what()<<std::endl;
    }
    save_integral_ = Eigen::VectorXd::Zero(nv);

    // Get base F/T data
    if(base_ft)
    {
        base_ft_frame_names = config["base_ft_frame_names"].as<std::vector<std::string>>();
        base_link_names = config["base_link_names"].as<std::vector<std::string>>();
        base_joint_names = config["base_joint_names"].as<std::vector<std::string>>();

        for(int i=0; i<base_ft_frame_names.size(); i++)
        {   
            const int id = pinocchio_model.getFrameId(base_ft_frame_names[i], pinocchio::FrameType::FIXED_JOINT);
            if(id != pinocchio_model.nframes)
            {
                std::cout<<"base_ft_frame_names["<<i<<"] : "<<base_ft_frame_names[i]<<std::endl;
                std::cout<<"base_ft_frame_id : "<<id<<std::endl;
                base_ft_frame_ids.push_back(pinocchio_model.getFrameId(base_ft_frame_names[i]));
            }
            else
            {   
                std::cout<<"ERROR: base_ft_joint_name is not valid"<<std::endl;
                std::cout<<"check base_ft_frame_names["<<i<<"] : "<<base_ft_frame_names[i]<<std::endl;
            }     
        }

        for(int i=0; i<base_link_names.size(); i++)
        {   
            const int id = pinocchio_model.getFrameId(base_link_names[i], pinocchio::FrameType::FIXED_JOINT);
            if(id != pinocchio_model.nframes)
            {
                std::cout<<"base_link_names["<<i<<"] : "<<base_link_names[i]<<std::endl;
                std::cout<<"base_link_id : "<<id<<std::endl;
                base_link_ids.push_back(pinocchio_model.getFrameId(base_link_names[i]));
            }
            else
            {   
                std::cout<<"ERROR: base_link_names is not valid"<<std::endl;
                std::cout<<"check base_link_names["<<i<<"] : "<<base_link_names[i]<<std::endl;
            }     
        }

        for(int i=0; i<base_joint_names.size(); i++)
        {   
            const int id = pinocchio_model.getJointId(base_joint_names[i]);
            if(id != pinocchio_model.njoints)
            {
                std::cout<<"base_joint_names["<<i<<"] : "<<base_joint_names[i]<<std::endl;
                std::cout<<"base_joint_id : "<<id<<std::endl;
                base_joint_ids.push_back(pinocchio_model.getJointId(base_joint_names[i]));
            }
            else
            {   
                std::cout<<"ERROR: base_joint_names is not valid"<<std::endl;
                std::cout<<"check base_joint_names["<<i<<"] : "<<base_joint_names[i]<<std::endl;
            }     
        }
    }

    if(is_controller_logging)
    {
        controller_logging_data.resize(nq, nv);
        controller_logging_data.setZero();
    }
}

void RobotState::update_config()
{
    try
    {
        config = YAML::LoadFile(config_path);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
}

Eigen::VectorXd RobotState::integrate(const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const double &dt)
{   
    Eigen::VectorXd qout(nq);
    qout.resize(nq); qout.setZero();
    pinocchio::integrate(pinocchio_model, q, dq*dt, qout);

    return qout;
}
// dxout = q2 - q1
Eigen::VectorXd RobotState::difference(const Eigen::VectorXd &q1, const Eigen::VectorXd &q2)
{   
    Eigen::VectorXd dqout(nv);
    dqout.setZero();

    pinocchio::difference(pinocchio_model, q1, q2, dqout);

    return dqout;
}

void RobotState::update_robot_state(const Gen3State & gen3_state)
{   
    q = gen3_state.converted_q;
    theta = gen3_state.converted_q;

    dq = gen3_state.joint_velocities;
    dtheta = gen3_state.joint_velocities;

    tau_J = gen3_state.joint_torques;
    joint_temperatures = gen3_state.joint_temperatures;
    pinocchio::forwardKinematics(pinocchio_model, pinocchio_data, q, dq);
    pinocchio::updateFramePlacements(pinocchio_model, pinocchio_data);

    update_DOB();
}

void RobotState::update_base_wrench(const Eigen::VectorXd& base_wrench_measured_arg)
{
    base_wrench_measured = base_wrench_measured_arg;
}

void RobotState::update_DOB()
{
    //momentum based observer
    Eigen::MatrixXd coriolis = this->get_coriolis(q,dq);
    Eigen::MatrixXd mass_matrix = this->get_mass(q);
    Eigen::VectorXd gravity = this->get_gravity(q);

    //for observer
    Eigen::MatrixXd KI(nv,nv); // diagonal gain matrix
    Eigen::VectorXd generalized_momentum(nv); // generalized momentum

    KI.setIdentity();
    KI = KI*200;
    generalized_momentum = mass_matrix * dq;

    //if starting generalized vel is 0
    Eigen::VectorXd temp_int = dt*(tau_J + coriolis.transpose()*dq - gravity + tau_ext);
    tau_ext = KI*(generalized_momentum -save_integral_ - temp_int);
    save_integral_ = save_integral_ + temp_int;

    if(base_ft)
    {
        // for estimated base wrench due to external force RNEA
        Eigen::VectorXd base_FT_nominal(6), nominal_ddq(nv);
        nominal_ddq = mass_matrix.inverse()*(tau_J - coriolis*dq-gravity + tau_ext);
        
        base_wrench_ext = this->get_base_ext_wrench(nominal_ddq);
    }
}

Eigen::MatrixXd RobotState::get_mass(const Eigen::VectorXd &q)
{
    pinocchio::crba(pinocchio_model,pinocchio_data,q); // compute mass matrix

    Eigen::MatrixXd temp = pinocchio_data.M;
    temp += temp.transpose().eval();
    temp.diagonal() = 0.5*temp.diagonal();

    return temp;
}

Eigen::MatrixXd RobotState::get_coriolis(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
{
    pinocchio::computeCoriolisMatrix(pinocchio_model,pinocchio_data, q, dq); // compute Coriolis matrix
    return pinocchio_data.C;
}

Eigen::MatrixXd RobotState::get_gravity(const Eigen::VectorXd &q)
{
    pinocchio::computeGeneralizedGravity(pinocchio_model, pinocchio_data, q);
    return pinocchio_data.g;
}

Eigen::MatrixXd RobotState::get_ee_jacobian_w(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK

    return this->get_ee_jacobian_w(ee_name);
}

Eigen::MatrixXd RobotState::get_ee_jacobian_w(const std::string & ee_name)
{
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index

    Eigen::MatrixXd Jacobian_b(6,pinocchio_model.nv); Jacobian_b.setZero();
    pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, ee_id, Jacobian_b);

    Eigen::Matrix<double,6,6> gen_rot; gen_rot.setZero();
    Eigen::MatrixXd Jacobian_w(6,pinocchio_model.nv); Jacobian_w.setZero();
    gen_rot.topLeftCorner(3,3) = SE3.rotation();
    gen_rot.bottomRightCorner(3,3) = SE3.rotation(); 

    Jacobian_w = gen_rot * Jacobian_b;

    return Jacobian_w;
}

Eigen::MatrixXd RobotState::get_ee_jacobian_b(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK
    
    return this->get_ee_jacobian_b(ee_name);
}

Eigen::MatrixXd RobotState::get_ee_jacobian_b(const std::string & ee_name)
{
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index

    Eigen::MatrixXd Jacobian_b(6,pinocchio_model.nv); Jacobian_b.setZero();
    pinocchio::computeFrameJacobian(pinocchio_model, pinocchio_data, q, ee_id, Jacobian_b);
    Jacobian_b;

    return Jacobian_b;   
}

Eigen::Affine3d RobotState::get_ee_pose(const Eigen::VectorXd &q, const std::string & ee_name)
{   
    pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q); // compute FK

    return this->get_ee_pose(ee_name);
}

Eigen::Affine3d RobotState::get_ee_pose(const std::string & ee_name)
{
    Eigen::Affine3d ee_htm;
    auto ee_id = pinocchio_model.getFrameId(ee_name,pinocchio::FrameType::FIXED_JOINT);

    // pinocchio::updateFramePlacement(pinocchio_model, pinocchio_data, ee_id); // compute FK
    // bool is_exist = pinocchio_model.existFrame(end_effector_name,pinocchio::FrameType::FIXED_JOINT);
    
    auto SE3 = pinocchio_data.oMf[ee_id]; // 1~7 joint frame index, 8 : end-effector index
    ee_htm = SE3.toHomogeneousMatrix();
    
    return ee_htm;
}

Eigen::VectorXd RobotState::get_base_ext_wrench(const Eigen::VectorXd &gacc)
{   
    Eigen::VectorXd base_wrench = this->get_base_nominal_wrench(gacc);
    base_wrench -= base_wrench_measured;

    return base_wrench;
}

Eigen::VectorXd RobotState::get_base_nominal_wrench(const Eigen::VectorXd &gacc)
{   
    // TODO : multiple base FT sensor measurement
    // pinocchio::framesForwardKinematics(pinocchio_model, pinocchio_data, q);
    pinocchio::rnea(pinocchio_model, pinocchio_data, q, dq, gacc);

    // wrench applied to the world frame w.r.t world
    std::vector<Eigen::VectorXd> base_joint_wrenchs;

    for(int i=0; i<base_joint_ids.size(); i++)
    {   
        int base_joint_id = base_joint_ids[i];
        Eigen::VectorXd base_joint_wrench(6); base_joint_wrench.setZero();
        auto SE3 = pinocchio_data.oMi[base_joint_id];
        auto f = pinocchio_data.f[base_joint_id];
        base_joint_wrench.head(3) = SE3.rotation()*f.linear();
        base_joint_wrench.tail(3) = skew(SE3.translation())*base_joint_wrench.head(3) + SE3.rotation()*f.angular();
        base_joint_wrenchs.push_back(base_joint_wrench);
    }

    // wrench applied to the world frame due to mount & base link
    std::vector<Eigen::Matrix<double,6,1>> base_ft_wrench(base_ft_frame_ids.size());
    std::fill(base_ft_wrench.begin(), base_ft_wrench.end(), Eigen::Matrix<double,6,1>::Zero());

    // wrench applied to the world frame due to mount w.r.t world
    for(int i=0; i<base_ft_frame_ids.size(); i++)
    {   
        int base_ft_frame_id = base_ft_frame_ids[i];
        auto SE3 = pinocchio_data.oMf[base_ft_frame_id];
        auto com = SE3.translation() + SE3.rotation()*pinocchio_model.frames[base_ft_frame_id].inertia.lever();
        auto mass = pinocchio_model.frames[base_ft_frame_id].inertia.mass();
        base_ft_wrench[i].head(3) -= mass * pinocchio_model.gravity.linear();
        base_ft_wrench[i].tail(3) -= mass * skew(com) * pinocchio_model.gravity.linear();

        // wrench transformation from sensor frame to world frame
        Eigen::VectorXd temp(6);
        temp.head(3) = SE3.rotation()*base_wrench_measured.head(3);
        temp.tail(3) = skew(SE3.translation())*SE3.rotation()*base_wrench_measured.head(3) 
                            + SE3.rotation()*base_wrench_measured.tail(3);
        base_wrench_measured = temp;
    }

    // wrench applied to the world frame due to base link w.r.t world
    for(int i=0; i<base_link_ids.size(); i++)
    {
        int base_link_id = base_link_ids[i];
        auto SE3 = pinocchio_data.oMf[base_link_id];
        auto com = SE3.translation() + SE3.rotation()*pinocchio_model.frames[base_link_id].inertia.lever();
        auto mass = pinocchio_model.frames[base_link_id].inertia.mass();
        base_ft_wrench[i].head(3) -= mass * pinocchio_model.gravity.linear();
        base_ft_wrench[i].tail(3) -= mass * skew(com) * pinocchio_model.gravity.linear();
    }


    // base wrench
    Eigen::VectorXd base_wrench(6); base_wrench.setZero();
    
    for(int i=0; i<base_ft_frame_ids.size(); i++)
    {
        base_wrench += base_ft_wrench[i] + base_joint_wrenchs[i];
    }

    return base_wrench;
}

Eigen::VectorXd RobotState::get_joint_nominal_torque(const Eigen::VectorXd &gacc)
{
    pinocchio::rnea(pinocchio_model, pinocchio_data, q, dq, gacc);

    Eigen::VectorXd joint_torque(7); joint_torque.setZero();
    joint_torque = pinocchio_data.tau.head(7);

    return joint_torque;
}


void RobotState::convert_q(Gen3State &gen3_state)
{   
    std::vector<double> q_vec;

    for(int i=0; i<nv; i++)
    {   
        //except for universe joint
        auto joint = pinocchio_model.joints[i+1];
        if(joint.nq() != joint.nv())
        {   
            q_vec.push_back(std::cos(gen3_state.joint_positions(i)));
            q_vec.push_back(std::sin(gen3_state.joint_positions(i)));
        }
        else
        {   
            q_vec.push_back(gen3_state.joint_positions(i));
        }
    }
    Eigen::VectorXd q_vec_eigen = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());

    if(q_vec_eigen.size() != nq)
    {
        std::cout<<"q_vec_eigen size is not equal to pinocchio_model_.nq"<<std::endl;
        std::cout<<"q_vec_eigen size : "<<q_vec_eigen.size()<<std::endl;
        std::cout<<"pinocchio_model_.nq : "<<nq<<std::endl;
    }

    gen3_state.converted_q = q_vec_eigen;
}

void RobotState::convert_q(Eigen::VectorXd &q_arg)
{
    std::vector<double> q_vec;

    for(int i=0; i<nv; i++)
    {   
        //except for universe joint
        auto joint = pinocchio_model.joints[i+1];
        if(joint.nq() != joint.nv())
        {   
            q_vec.push_back(std::cos(q_arg(i)));
            q_vec.push_back(std::sin(q_arg(i)));
        }
        else
        {   
            q_vec.push_back(q_arg(i));
        }
    }
    q_arg= Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(q_vec.data(), q_vec.size());
}


MujocoApplyForce RobotState::convert_force_frame(const MujocoApplyForce & mujoco_apply_force)
{
    const int num_contacts = mujoco_apply_force.contact_link_id.size();
    converted_mujoco_apply_force.contact_link_id = mujoco_apply_force.contact_link_id;
    converted_mujoco_apply_force.resize(num_contacts);
    converted_mujoco_apply_force.is_visualize = mujoco_apply_force.is_visualize;

    
    for(int i=0; i<num_contacts; i++)
    {
        const int contact_link_id = mujoco_apply_force.contact_link_id[i];
        auto se3 = pinocchio_data.oMi[contact_link_id];
        converted_mujoco_apply_force.contact_point[i] = se3.translation() + se3.rotation()*mujoco_apply_force.contact_point[i];
        converted_mujoco_apply_force.contact_link_com[i] = se3.translation() + se3.rotation()*mujoco_apply_force.contact_link_com[i];

        if(mujoco_apply_force.k_env > 0.0)
        {   
            // std::cout<<"spring!"<<std::endl;   
            converted_mujoco_apply_force.contact_force[i] = - mujoco_apply_force.K_env[i]*(converted_mujoco_apply_force.contact_point[i] - mujoco_apply_force.env_location[i]);
        }
        else
        {   
            // std::cout<<"no spring!"<<std::endl;
            converted_mujoco_apply_force.contact_force[i] = se3.rotation()*mujoco_apply_force.contact_force[i];
        }

        // std::cout<<"converted_mujoco_apply_force.contact_force : "<<std::endl;
        // std::cout<<converted_mujoco_apply_force.contact_force[i].transpose()<<std::endl;
        // std::cout<<"norm : " << converted_mujoco_apply_force.contact_force[i].norm()<<std::endl;
        // std::cout<<"converted_mujoco_apply_force.contact_point : "<<std::endl;
        // std::cout<<converted_mujoco_apply_force.contact_point[i].transpose()<<std::endl;
    }

    

    

    return converted_mujoco_apply_force;
}