#include "node/kinova_controller_node.hpp"

#include <memory>
#include <utility>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "utils/process_settings.hpp"
#include "utils/lifecycle_autostart.hpp"

int main(int argc, char * argv[])
{
  utils::ProcessSettings settings;
  if (!settings.init(argc, argv)) {
    return EXIT_FAILURE;
  }

  int32_t ret = 0;
  try {
    // configure process real-time settings
    if (settings.configure_child_threads) {
      // process child threads created by ROS nodes will inherit the settings
      settings.configure_process();
    }
    rclcpp::init(argc, argv);

    // Create a static executor
    rclcpp::executors::StaticSingleThreadedExecutor exec;

    // Create yaml config
    std::string package_path = __FILE__;
    std::string erase_str = "src/main.cpp";
    std::string::size_type start = package_path.find(erase_str);
    if (start != std::string::npos)
        package_path.erase(start, erase_str.length());
    std::string config_path = package_path+"config/kinova_controller_config.yaml";
    std::cout<<"config yaml file path : " << config_path << std::endl;
    YAML::Node config = YAML::LoadFile(config_path);


    // Create pinocchio model
    std::string pinocchio_urdf_path = config["pinocchio_urdf_path"].as<std::string>();
    pinocchio::Model pinocchio_model;
    pinocchio::urdf::buildModel(pinocchio_urdf_path,pinocchio_model);
    pinocchio_model.gravity.linear(pinocchio::Model::gravity981);

    // Create RobotState
    std::shared_ptr<RobotState> robot_state = std::make_shared<RobotState>(config, pinocchio_model, config_path);

    // For simulation
    std::unique_ptr<mj::Simulate> sim;
    if(robot_state->is_simulation)
    {
        // init GLFW
        if (!Glfw().glfwInit()) {
            mju_error("could not initialize GLFW");
        }

        sim = std::make_unique<mj::Simulate>();
        
        sim->ui0_enable = 0; // fold left ui
        sim->ui1_enable = 0; // fold right ui
        sim->info = 1; // show info

        std::thread([&sim]() {sim->renderloop();}).detach();
    }

    // Create KinovaControllerNode
    const auto kinova_controller_node_ptr = std::make_shared<KinovaControllerNode>("kinova_controller");
    kinova_controller_node_ptr->assign_robot_state(robot_state);

    if(robot_state->is_simulation)
    {
      kinova_controller_node_ptr->create_simulation_robot(sim.get());
    }
    else
    {
      kinova_controller_node_ptr->create_real_robot();
    }

  
    exec.add_node(kinova_controller_node_ptr->get_node_base_interface());

    // configure process real-time settings
    if (!settings.configure_child_threads) {
      // process child threads created by ROS nodes will NOT inherit the settings
      settings.configure_process();
    }

    if (settings.auto_start_nodes) {
      utils::autostart(*kinova_controller_node_ptr);
    }
    exec.spin();

    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_INFO(rclcpp::get_logger("kinova_controller"), e.what());
    ret = 2;
  } catch (...) {
    RCLCPP_INFO(
      rclcpp::get_logger("kinova_controller"), "Unknown exception caught. "
      "Exiting...");
    ret = -1;
  }
  return ret;
}
