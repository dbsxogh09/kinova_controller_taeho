#include "kinova_controller/data/mujoco_apply_force.hpp"
void MujocoApplyForce::clear()
{
    contact_point.clear();
    contact_force.clear();
    contact_link_com.clear();
    contact_link_id.clear();
}

void MujocoApplyForce::resize(const int &size)
{
    contact_point.resize(size);
    contact_force.resize(size);
    contact_link_com.resize(size);
    contact_link_id.resize(size);
}

void MujocoApplyForce::print()
{
    std::cout<<"contact_point: "<<std::endl;
    for(int i=0; i<contact_point.size(); i++)
    {
        std::cout<<contact_point[i].transpose()<<std::endl;
    }
    std::cout<<"contact_force: "<<std::endl;
    for(int i=0; i<contact_force.size(); i++)
    {
        std::cout<<contact_force[i].transpose()<<std::endl;
    }
    std::cout<<"contact_link_com: "<<std::endl;
    for(int i=0; i<contact_link_com.size(); i++)
    {
        std::cout<<contact_link_com[i].transpose()<<std::endl;
    }
    std::cout<<"contact_link_id: "<<std::endl;
    for(int i=0; i<contact_link_id.size(); i++)
    {
        std::cout<<contact_link_id[i]<<std::endl;
    }
}