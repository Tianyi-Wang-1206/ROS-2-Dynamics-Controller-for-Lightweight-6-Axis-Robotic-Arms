cmake_minimum_required(VERSION 3.5)
project(trac_ik_lib)

if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
    set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)

find_package(NLopt REQUIRED)

find_package(PkgConfig)
pkg_check_modules(Eigen REQUIRED eigen3)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

set(DEPS
  rclcpp
  geometry_msgs
  urdf
  kdl_parser
  NLopt
  )



add_library(trac_ik SHARED src/kdl_tl.cpp src/nlopt_ik.cpp src/trac_ik.cpp)
target_link_libraries(trac_ik ${pkg_nlopt_LIBRARIES})
target_include_directories(trac_ik PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

ament_target_dependencies( trac_ik  ${DEPS})


install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS trac_ik
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_export_libraries(
  trac_ik
)
ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPS}
  )

ament_export_include_directories(
  include
)


ament_package()
