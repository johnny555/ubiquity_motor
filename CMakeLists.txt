cmake_minimum_required(VERSION 3.8)
project(ubiquity_motor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(controller_manager REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS
	thread
	atomic)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Ubiquity motor library
add_library(
  ${PROJECT_NAME}
  SHARED
  src/motor_system.cpp
  src/motor_interface.cpp
  src/motor_message.cpp
  src/motor_serial.cpp
  src/motor_diagnostics_broadcaster.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  serial
  std_msgs
  sensor_msgs
  hardware_interface
  pluginlib
  realtime_tools
  controller_manager
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
)

# Probe robot
add_executable(probe_robot src/probe_robot.cpp)
ament_target_dependencies(probe_robot
  serial
)
target_link_libraries(probe_robot
${PROJECT_NAME}
 ${Boost_LIBRARIES}
)

# TODO(sam): generate custom Leds message

pluginlib_export_plugin_description_file(hardware_interface ubiquity_motor.xml)
pluginlib_export_plugin_description_file(controller_interface ubiquity_motor_diagnostics_broadcaster.xml)

install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib
)

install(TARGETS 
  probe_robot
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

  # TODO(sam): Fix issues and turn on linters
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_copyright
    ament_cmake_cppcheck
    ament_cmake_cpplint
    ament_cmake_lint_cmake
    ament_cmake_flake8
    ament_cmake_pep257
    ament_cmake_xmllint
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()
  include_directories(test)

  ament_add_gtest(motor_message_test test/motor_message_test.cpp)
	target_link_libraries(motor_message_test ${PROJECT_NAME})

	ament_add_gtest(motor_serial_test test/motor_serial_test.cpp)
	target_link_libraries(motor_serial_test ${PROJECT_NAME} util)

	ament_add_gtest(shared_queue_test test/shared_queue_test.cpp)
	target_link_libraries(shared_queue_test ${PROJECT_NAME})

	# ament_add_gtest(motor_interface_test test/motor_interface_test.cc )
	# target_link_libraries(motor_interface_test ${PROJECT_NAME} util)

endif()

ament_package()