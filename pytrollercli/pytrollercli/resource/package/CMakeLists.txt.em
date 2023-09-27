cmake_minimum_required(VERSION 3.16)
project(@(pytroller_name) LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/@(pytroller_name)_logic.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/@(pytroller_name)_logic.h
  COMMAND cython -3 -+ ${CMAKE_CURRENT_SOURCE_DIR}/src/@(pytroller_name)_logic.pyx
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/@(pytroller_name)_logic.pyx
)

generate_parameter_library(
  @(pytroller_name)_parameters
  src/@(pytroller_name)_parameters.yaml
)

add_library(@(pytroller_name) SHARED
  src/@(pytroller_name).cpp
  src/@(pytroller_name)_logic.cpp
  src/@(pytroller_name)_logic.h
)
target_compile_features(@(pytroller_name) PUBLIC cxx_std_17)
target_include_directories(@(pytroller_name) PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/@(pytroller_name)>
)
target_link_libraries(@(pytroller_name) PUBLIC
  @(pytroller_name)_parameters
  ${PYTHON_LIBRARIES}
)
ament_target_dependencies(@(pytroller_name) PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(@(pytroller_name) PRIVATE "PYTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_rostest_with_parameters_gmock(
    test_load_@(pytroller_name)
    test/test_load_@(pytroller_name).cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )
  target_link_libraries(test_load_@(pytroller_name)
    @(pytroller_name)
  )
  ament_target_dependencies(test_load_@(pytroller_name)
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

#   ament_add_gmock(test_@(pytroller_name)
#     test/test_@(pytroller_name).cpp
#   )
#   target_link_libraries(test_@(pytroller_name)
#     @(pytroller_name)
#   )

endif()

install(
  DIRECTORY include/
  DESTINATION include/@(pytroller_name)
)
install(
  TARGETS
    @(pytroller_name)
    @(pytroller_name)_parameters
  EXPORT export_@(pytroller_name)
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_@(pytroller_name) HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()