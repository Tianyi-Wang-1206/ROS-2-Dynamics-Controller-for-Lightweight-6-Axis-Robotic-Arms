cmake_minimum_required(VERSION 3.5)
project(trac_ik_kinematics_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(class_loader REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(trac_ik_lib REQUIRED)

set(DEPS
  rclcpp
  moveit_core
  pluginlib
  class_loader
  urdf
  tf2_kdl
  generate_parameter_library
  trac_ik_lib
  )

set(TRAC_IK_PLUGIN_LIB trac_ik_kinematics_plugin)
set(TRAC_IK_PARAM_LIB trac_ik_kinematics_parameters)

generate_parameter_library(${TRAC_IK_PARAM_LIB} src/trac_ik_kinematics_parameters.yaml)

add_library(${TRAC_IK_PLUGIN_LIB} SHARED src/trac_ik_kinematics_plugin.cpp)
target_link_libraries(${TRAC_IK_PLUGIN_LIB} ${TRAC_IK_PARAM_LIB})

target_include_directories(${TRAC_IK_PLUGIN_LIB} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )
ament_target_dependencies(${TRAC_IK_PLUGIN_LIB} ${DEPS})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${TRAC_IK_PLUGIN_LIB} ${TRAC_IK_PARAM_LIB}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it. trac_ik_kinematics_plugin
target_compile_definitions(${TRAC_IK_PLUGIN_LIB} PRIVATE "TRAC_IK_KINEMATICS_PLUGIN_BUILDING_DLL")

pluginlib_export_plugin_description_file(moveit_core trac_ik_kinematics_description.xml)

ament_export_libraries(${TRAC_IK_PLUGIN_LIB})
ament_export_targets( export_${PROJECT_NAME} )
ament_export_dependencies( ${DEPS} )
ament_export_include_directories(include)

ament_package()
