cmake_minimum_required(VERSION 3.8)
project(aerosimian)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Build the component as a SHARED library
add_library(state_estimator_node SHARED
  src/state_estimator.cpp
)

ament_target_dependencies(state_estimator_node
  rclcpp
  rclcpp_components
  geometry_msgs
)

include(FetchContent)
FetchContent_Declare(
  moteus
  GIT_REPOSITORY https://github.com/mjbots/moteus.git
  GIT_TAG        f14f789fefeae9caf185b950cf4e32df01d022ea
)

FetchContent_MakeAvailable(moteus)



# Register the component so component_container can load it
rclcpp_components_register_nodes(state_estimator_node
  "aerosimian::AeroSimianStateEstimatorComponent"
)

# Install the library to lib/<package>
install(TARGETS state_estimator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
target_link_libraries(state_estimator_node moteus::cpp)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


