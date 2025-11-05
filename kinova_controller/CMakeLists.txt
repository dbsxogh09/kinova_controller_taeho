cmake_minimum_required(VERSION 3.5)
project(kinova_controller)

add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)
# /usr, /usr/local 아래는 찾지 않도록(필요 시 범위 조절)
set(CMAKE_IGNORE_PREFIX_PATH "/usr;/usr/local")

# 빌드/설치 산출물의 런타임 경로 고정
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH ON)
list(APPEND CMAKE_INSTALL_RPATH
  "/opt/ros/foxy/lib/x86_64-linux-gnu"
  "/opt/ros/foxy/lib"
  "/opt/openrobots/lib"
  "${MUJOCO_LIB_DIR}"

)

enable_language(C)
enable_language(CXX)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)


find_package(ament_index_cpp)

find_package(robot_state_msgs)
find_package(opt_ctrl_msgs)
find_package(mcp_ep_msgs)
find_package(controller_interface_msgs)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# for kortex API
set(KORTEX_LIB_DIR "${PROJECT_SOURCE_DIR}/lib/kortex_api")
set(KORTEX_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/kortex_api")

include_directories(${KORTEX_INCLUDE_DIR})
include_directories(${KORTEX_INCLUDE_DIR}/client)
include_directories(${KORTEX_INCLUDE_DIR}/common)
include_directories(${KORTEX_INCLUDE_DIR}/messages)
include_directories(${KORTEX_INCLUDE_DIR}/client_stubs)
include_directories(${KORTEX_INCLUDE_DIR}/cxxopts/)
include_directories(${KORTEX_INCLUDE_DIR}/messages/)
link_libraries(${KORTEX_LIB_DIR}/libKortexApiCpp.a)
add_definitions(-D_OS_UNIX)

# for Mujoco start
set(MUJOCO_PATH "${PROJECT_SOURCE_DIR}/include/mujoco-2.3.1")
set(SIMULATE_PATH "${MUJOCO_PATH}/simulate")

find_package(OpenGL REQUIRED COMPONENTS OpenGL EGL)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Curses REQUIRED)
find_package(glfw3 REQUIRED)

find_library(MUJOCO
  NAMES libmujoco.so.2.3.1
  PATHS ${MUJOCO_PATH}/lib
  REQUIRED
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${rclcpp_INCLUDE_DIRS}
  ${robot_state_msgs_INCLUDE_DIRS}
  ${opt_ctrl_msgs_INCLUDE_DIRS}
  ${mcp_ep_msgs_INCLUDE_DIRS}
  ${controller_interface_msgs_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_components_INCLUDE_DIRS}
)

link_libraries(
  Eigen3::Eigen
  pinocchio::pinocchio
  ${YAML_CPP_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${robot_state_msgs_LIBRARIES}
  ${opt_ctrl_msgs_LIBRARIES}
  ${mcp_ep_msgs_LIBRARIES}
  ${controller_interface_msgs_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${lifecycle_msgs_LIBRARIES}
  ${rclcpp_components_LIBRARIES}
)

add_library(data STATIC
  src/kinova_controller/data/controller_state.cpp
  src/kinova_controller/data/gen3_state.cpp
  src/kinova_controller/data/mcp_ep_visualization.cpp
  src/kinova_controller/data/mujoco_apply_force.cpp
  src/kinova_controller/data/nominal_plant.cpp
  src/kinova_controller/data/opt_data.cpp
  src/kinova_controller/data/robot_state.cpp
  src/kinova_controller/data/energy_tank_data.cpp
  src/kinova_controller/data/controller_logging_data.cpp
  )

add_library(lodepng STATIC ${SIMULATE_PATH}/lodepng.h ${SIMULATE_PATH}/lodepng.cpp)
add_library(libsimulate STATIC)
target_sources(
  libsimulate
  PUBLIC  ${SIMULATE_PATH}/simulate.h
  PRIVATE ${SIMULATE_PATH}/simulate.cc
          ${SIMULATE_PATH}/array_safety.h
          ${SIMULATE_PATH}/uitools.h
          ${SIMULATE_PATH}/uitools.cc
          ${SIMULATE_PATH}/glfw_dispatch.h
          ${SIMULATE_PATH}/glfw_dispatch.cc
)
target_link_libraries(libsimulate PUBLIC lodepng ${MUJOCO} ${rclcpp_LIBRARIES} ${mcp_ep_msgs_LIBRARIES})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
target_compile_options(libsimulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_include_directories(libsimulate 
  PUBLIC 
    ${SIMULATE_PATH}
    ${MUJOCO_PATH}/include
    ${MUJOCO_PATH}/simulate
    ${MUJOCO_PATH}/sample
    ${rclcpp_INCLUDE_DIRS}
    ${mcp_ep_msgs_INCLUDE_DIRS}
)
target_compile_features(libsimulate PRIVATE cxx_std_17)
link_libraries(libsimulate
                lodepng
                glfw
                ${GLFW3_LIBRARY} 
                libsimulate
                ${CURSES_LIBRARY}
                ${MUJOCO}
                data
)
include_directories(
  ${MUJOCO_PATH}/include/mujoco
  ${MUJOCO_PATH}/include
  ${MUJOCO_PATH}/simulate
  ${MUJOCO_PATH}/sample
  ${GLFW_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR}
  ${CURSES_INCLUDE_DIR}
)
# for Mujoco end

add_library(utils STATIC
  src/utils/memory_lock.cpp
  src/utils/rt_thread.cpp
)

add_library(sensors STATIC
  src/kinova_controller/sensors/ati_ft.cpp
)

add_library(tank STATIC
  src/kinova_controller/energy_tank/energy_tank_base.cpp
  src/kinova_controller/energy_tank/energy_tank.cpp)
target_link_libraries(tank data)

add_library(kinova_controller STATIC
  src/kinova_controller/controller/controller.cpp
  src/kinova_controller/controller/controller_gain.cpp
  src/kinova_controller/filter/lpf.cpp
)
target_link_libraries(kinova_controller data tank)

add_library(robot STATIC
  src/kinova_controller/robot/kinova_gen3.cpp
  src/kinova_controller/robot/kinova_gen3_simulation.cpp
)
target_link_libraries(robot kinova_controller data tank sensors)

add_library(node STATIC
  src/node/kinova_controller_node.cpp)
target_link_libraries(node robot kinova_controller data tank sensors)


add_executable(run 
  src/main.cpp
)

target_link_libraries(run robot data utils node kinova_controller tank sensors)

# install executables
install(TARGETS
  run
  DESTINATION lib/${PROJECT_NAME})

# install utils include
install(
  DIRECTORY include/
  DESTINATION include
)

# install utils lib
install(TARGETS utils 
  EXPORT export_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include)



# install launch
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()