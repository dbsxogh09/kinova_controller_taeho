#include "kinova_controller/data/mcp_ep_visualization.hpp"

void MCP_EP_Visualization::print()
{
    std::cout << "particle_pos: " << std::endl;
    for (int i = 0; i < particle_pos.size(); i++)
    {
        for (int j = 0; j < particle_pos[i].size(); j++)
        {
            std::cout << particle_pos[i][j].transpose() << std::endl;
        }
    }
    std::cout << "particle_force: " << std::endl;
    for (int i = 0; i < particle_force.size(); i++)
    {
        for (int j = 0; j < particle_force[i].size(); j++)
        {
            std::cout << particle_force[i][j].transpose() << std::endl;
        }
    }
    std::cout << "points: " << std::endl;
    for (int i = 0; i < points.size(); i++)
    {
        std::cout << points[i].transpose() << std::endl;
    }
    std::cout << "forces: " << std::endl;
    for (int i = 0; i < forces.size(); i++)
    {
        std::cout << forces[i].transpose() << std::endl;
    }
    std::cout << "exploit_particle_num: " << std::endl;
    for (int i = 0; i < exploit_particle_num.size(); i++)
    {
        std::cout << exploit_particle_num[i] << std::endl;
    }

}
