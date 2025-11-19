cmake_minimum_required(VERSION 3.16)
project(aerosimian)

# --- ROS / deps ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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
add_library(state_estimator_node SHARED
  src/state_estimator.cpp
)

# Ensure modern C++ standard
target_compile_features(state_estimator_node PUBLIC cxx_std_17)

# Link Eigen (plain form to match ament_target_dependencies)
target_link_libraries(state_estimator_node Eigen3::Eigen)

# ament dependencies (this will call target_link_libraries in plain form)
ament_target_dependencies(state_estimator_node
  rclcpp
  rclcpp_components
  geometry_msgs
)

# Try to link moteus if an exported target exists (plain link form)
if (TARGET moteus::cpp)
  target_link_libraries(state_estimator_node moteus::cpp)
elseif (TARGET moteus)
  target_link_libraries(state_estimator_node moteus)
elseif (TARGET mjbots::moteus)
  target_link_libraries(state_estimator_node mjbots::moteus)
else()
  message(WARNING "Could not find an obvious moteus CMake target. Inspect the moteus repo CMake to pick the correct target name or vendor it into src/.")
endif()

# Register as a component
rclcpp_components_register_nodes(state_estimator_node
  "aerosimian::AeroSimianStateEstimatorComponent"
)

install(TARGETS state_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# driver executable
add_executable(driver src/driver.cpp)
target_compile_features(driver PUBLIC cxx_std_17)

# link moteus for driver (plain form)
if (TARGET moteus::cpp)
  target_link_libraries(driver moteus::cpp)
elseif (TARGET moteus)
  target_link_libraries(driver moteus)
elseif (TARGET mjbots::moteus)
  target_link_libraries(driver mjbots::moteus)
endif()

ament_target_dependencies(driver rclcpp)

install(TARGETS driver
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

