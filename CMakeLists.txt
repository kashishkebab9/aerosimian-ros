cmake_minimum_required(VERSION 3.16)
project(aerosimian)

# --- ROS / deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/AeroSimianState.msg"
  "msg/AeroSimianControl.msg"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs
)

# --- FetchContent (optional) ---
include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        f14f789fefeae9caf185b950cf4e32df01d022ea
)
# This may attempt to clone during configure. If you prefer to vendor the repo into src/, skip FetchContent and use add_subdirectory instead.
FetchContent_MakeAvailable(moteus)

# --- Build the component as a shared library ---
add_library(theta_estimator_node SHARED
  src/AeroSimianThetaStateEstimator.cpp
)

# Ensure modern C++ standard
target_compile_features(theta_estimator_node PUBLIC cxx_std_17)

# Link Eigen (plain form to match ament_target_dependencies)
target_link_libraries(theta_estimator_node Eigen3::Eigen)

# ament dependencies (this will call target_link_libraries in plain form)
ament_target_dependencies(theta_estimator_node
  rclcpp
  rclcpp_components
  geometry_msgs
)

rosidl_target_interfaces(theta_estimator_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Try to link moteus if an exported target exists (plain link form)
if (TARGET moteus::cpp)
  target_link_libraries(theta_estimator_node moteus::cpp)
elseif (TARGET moteus)
  target_link_libraries(theta_estimator_node moteus)
elseif (TARGET mjbots::moteus)
  target_link_libraries(theta_estimator_node mjbots::moteus)
else()
  message(WARNING "Could not find an obvious moteus CMake target. Inspect the moteus repo CMake to pick the correct target name or vendor it into src/.")
endif()

# Register as a component
rclcpp_components_register_nodes(theta_estimator_node
  "aerosimian::AeroSimianThetaStateEstimatorComponent"
)

install(TARGETS theta_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# driver executable
add_executable(driver src/driver.cpp)
target_compile_features(driver PUBLIC cxx_std_17)

add_executable(aerosimian_full_state_node
  src/AeroSimianFullStateEstimator.cpp
)
target_compile_features(aerosimian_full_state_node PUBLIC cxx_std_17)

ament_target_dependencies(aerosimian_full_state_node
  rclcpp
  std_msgs
)
rosidl_target_interfaces(aerosimian_full_state_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_link_libraries(aerosimian_full_state_node Eigen3::Eigen)
# link moteus for driver (plain form)
if (TARGET moteus::cpp)
  target_link_libraries(driver moteus::cpp)
  target_link_libraries(aerosimian_full_state_node moteus::cpp)
elseif (TARGET moteus)
  target_link_libraries(driver moteus)
  target_link_libraries(aerosimian_full_state_node moteus)
elseif (TARGET mjbots::moteus)
  target_link_libraries(driver mjbots::moteus)
  target_link_libraries(aerosimian_full_state_node mjbots::moteus)
endif()

include_directories(include)

ament_target_dependencies(driver rclcpp)
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS aerosimian_full_state_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()


