cmake_minimum_required(VERSION 3.16)
project(abb_cell_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/send_test_trajectory.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_mock_trajectory.py TIMEOUT 120)
endif()

ament_package()
