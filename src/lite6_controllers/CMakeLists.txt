cmake_minimum_required(VERSION 3.8)
project(lite6_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Compile the controller library
add_library(${PROJECT_NAME} SHARED src/ctc_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  realtime_tools
  trajectory_msgs
  ament_index_cpp
  std_msgs
)
# Link Pinocchio library
target_link_libraries(${PROJECT_NAME} pinocchio::pinocchio)

# 3. Export plugin description file
pluginlib_export_plugin_description_file(controller_interface lite6_controllers.xml)

# 4. Install the library and header files
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(FILES lite6_controllers.xml
  DESTINATION share/${PROJECT_NAME}
)

# 5. Ament package
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp controller_interface hardware_interface pluginlib realtime_tools trajectory_msgs ament_index_cpp pinocchio)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()