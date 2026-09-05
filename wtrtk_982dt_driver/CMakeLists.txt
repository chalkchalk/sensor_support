cmake_minimum_required(VERSION 3.8)
project(wtrtk_982dt_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/GnssAttitude.msg"
  DEPENDENCIES std_msgs
)

# Keep application modules in a separate Python namespace.  ROSIDL owns the
# wtrtk_982dt_driver Python namespace used by generated messages.
ament_python_install_package(gps_ntrip_py)

install(PROGRAMS
  scripts/gps_ntrip_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  set(_pytest_environment "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}")
  ament_add_pytest_test(test_hpr test/test_hpr.py
    APPEND_ENV ${_pytest_environment}
  )
  ament_add_pytest_test(test_nmea test/test_nmea.py
    APPEND_ENV ${_pytest_environment}
  )
  ament_add_pytest_test(test_ntrip test/test_ntrip.py
    APPEND_ENV ${_pytest_environment}
  )
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
