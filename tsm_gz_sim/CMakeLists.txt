cmake_minimum_required(VERSION 3.8)
project(tsm_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##################################################
# Find all dependencies and generate ament index #
##################################################
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

####################
# Generate library #
####################
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/tsm_gz_sim_node.cpp
  src/keyboard_node.cpp
)
target_link_libraries(${PROJECT_NAME}
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tsm::TsmGzSimNode"
  EXECUTABLE tsm_gz_sim_node
)
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "tsm::KeyboardNode"
  EXECUTABLE keyboard_node
)

###########
# Install #
###########
ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  sdf
  launch
  config
  rviz
)
