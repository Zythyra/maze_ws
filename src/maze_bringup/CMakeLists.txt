cmake_minimum_required(VERSION 3.8)
project(maze_bringup)
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch launch_support config rviz world DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/teleop_keyboard.py scripts/velocity_guard.py
  scripts/maze_odometry.py scripts/polyline_follower.py
  DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/polyline_core.py scripts/return_planner.py scripts/branch_map.py scripts/terminal_start.py
  scripts/motion_reference.py scripts/differential_mpc.py DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_polyline test/test_polyline.py)
  ament_add_pytest_test(test_return_planner test/test_return_planner.py)
  ament_add_pytest_test(test_branch_map test/test_branch_map.py)
  ament_add_pytest_test(test_motion_reference test/test_motion_reference.py)
  ament_add_pytest_test(test_differential_mpc test/test_differential_mpc.py)
  ament_add_pytest_test(test_motion_handoff test/test_motion_handoff.py)
  ament_add_pytest_test(test_terminal_start test/test_terminal_start.py)
endif()
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/turtlebot3_model.dsv.in")
ament_package()
