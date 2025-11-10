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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()


