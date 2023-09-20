cmake_minimum_required(VERSION 3.16)
project(pytroller LANGUAGES CXX)

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
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/pytroller_logic.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/pytroller_logic.h
  COMMAND cython -3 -+ ${CMAKE_CURRENT_SOURCE_DIR}/src/pytroller_logic.pyx
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src
  DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pytroller_logic.pyx
)

generate_parameter_library(
  pytroller_parameters
  src/pytroller_parameters.yaml
)

add_library(pytroller SHARED
  src/pytroller.cpp
  src/pytroller_logic.cpp
  src/pytroller_logic.h
)
target_compile_features(pytroller PUBLIC cxx_std_17)
target_include_directories(pytroller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/pytroller>
)
target_link_libraries(pytroller PUBLIC
  pytroller_parameters
  ${PYTHON_LIBRARIES}
)
ament_target_dependencies(pytroller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(pytroller PRIVATE "PYTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface controller_plugin.xml)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(hardware_interface REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_rostest_with_parameters_gmock(
    test_load_pytroller
    test/test_load_pytroller.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_params.yaml
  )
  target_link_libraries(test_load_pytroller
    pytroller
  )
  ament_target_dependencies(test_load_pytroller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

#   ament_add_gmock(test_pytroller
#     test/test_pytroller.cpp
#   )
#   target_link_libraries(test_pytroller
#     pytroller
#   )

endif()

install(
  DIRECTORY include/
  DESTINATION include/pytroller
)
install(
  TARGETS
    pytroller
    pytroller_parameters
  EXPORT export_pytroller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_pytroller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()