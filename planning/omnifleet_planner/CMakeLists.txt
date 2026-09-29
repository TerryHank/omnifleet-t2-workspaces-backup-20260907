cmake_minimum_required(VERSION 3.8)
project(omnifleet_planner)

find_package(ament_cmake REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
add_library(omnifleet_hot_rpp SHARED src/hot_rpp.cpp)
target_include_directories(omnifleet_hot_rpp PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(omnifleet_hot_rpp nav2_regulated_pure_pursuit_controller nav2_core pluginlib)
pluginlib_export_plugin_description_file(nav2_core hot_rpp.xml)
install(TARGETS omnifleet_hot_rpp LIBRARY DESTINATION lib)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(omnifleet_navigation_policy_bt_node SHARED src/navigation_policy.cpp)
target_compile_definitions(omnifleet_navigation_policy_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(omnifleet_navigation_policy_bt_node behaviortree_cpp_v3 rclcpp geometry_msgs)
install(TARGETS omnifleet_navigation_policy_bt_node LIBRARY DESTINATION lib)
add_library(omnifleet_continuous_route_bt_node SHARED src/continuous_route_plugin.cpp)
target_compile_definitions(omnifleet_continuous_route_bt_node PRIVATE BT_PLUGIN_EXPORT)
ament_target_dependencies(omnifleet_continuous_route_bt_node behaviortree_cpp_v3 rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS omnifleet_continuous_route_bt_node LIBRARY DESTINATION lib)

add_library(footprint_safe_goal_checker SHARED
  src/footprint_safe_goal_checker.cpp)
target_include_directories(footprint_safe_goal_checker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(footprint_safe_goal_checker
  std_msgs
  nav2_controller
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  tf2)
pluginlib_export_plugin_description_file(nav2_core footprint_safe_goal_checker.xml)

install(DIRECTORY config launch behavior_trees
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(TARGETS footprint_safe_goal_checker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  add_executable(test_continuous_route src/test_continuous_route.cpp)
  ament_target_dependencies(test_continuous_route behaviortree_cpp_v3 rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
  add_test(NAME continuous_route COMMAND test_continuous_route)
  set_tests_properties(continuous_route PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174" TIMEOUT 30)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_goal_footprint_cost test/test_goal_footprint_cost.cpp)
  target_include_directories(test_goal_footprint_cost PRIVATE include)
  ament_target_dependencies(test_goal_footprint_cost nav2_costmap_2d geometry_msgs nav2_controller rclcpp)
  target_link_libraries(test_goal_footprint_cost footprint_safe_goal_checker)
  set_tests_properties(test_goal_footprint_cost PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=178;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_hot_rpp test/test_hot_rpp.cpp)
  target_include_directories(test_hot_rpp PRIVATE include)
  ament_target_dependencies(test_hot_rpp nav2_regulated_pure_pursuit_controller nav2_core)
  add_executable(test_navigation_policy src/test_policy.cpp)
  ament_target_dependencies(test_navigation_policy behaviortree_cpp_v3 rclcpp geometry_msgs)
  add_test(NAME navigation_policy COMMAND test_navigation_policy)
  set_tests_properties(navigation_policy PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_t2_stack test/test_t2_nav2_stack.py)
  ament_add_pytest_test(
    test_footprint_safe_goal_checker_config
    test/test_footprint_safe_goal_checker_config.py)
endif()


find_package(nav2_mppi_controller REQUIRED)
add_library(ordered_mppi SHARED src/ordered_mppi.cpp)
target_include_directories(ordered_mppi PRIVATE include)
ament_target_dependencies(ordered_mppi nav2_mppi_controller nav2_core pluginlib)
target_link_libraries(ordered_mppi footprint_safe_goal_checker)
pluginlib_export_plugin_description_file(nav2_core ordered_mppi.xml)
install(TARGETS ordered_mppi LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  add_executable(test_arc_progress src/test_arc_progress.cpp)
  target_include_directories(test_arc_progress PRIVATE include)
  ament_target_dependencies(test_arc_progress nav_msgs)
  add_test(NAME arc_progress COMMAND test_arc_progress)
  add_executable(test_ordered_mppi src/test_ordered_mppi.cpp)
  target_include_directories(test_ordered_mppi PRIVATE include)
  ament_target_dependencies(test_ordered_mppi nav2_mppi_controller nav2_core pluginlib)
  target_link_libraries(test_ordered_mppi footprint_safe_goal_checker)
  add_test(NAME ordered_mppi COMMAND test_ordered_mppi ${CMAKE_CURRENT_SOURCE_DIR}/config/nav2_t2.yaml)
  set_tests_properties(ordered_mppi PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174" TIMEOUT 30)
endif()
add_executable(test_ordered_progress src/test_ordered_progress.cpp)
target_include_directories(test_ordered_progress PRIVATE include)
ament_target_dependencies(test_ordered_progress nav_msgs)
add_test(NAME ordered_progress COMMAND test_ordered_progress ${CMAKE_CURRENT_SOURCE_DIR}/test/data/route207.txt)
add_executable(test_route_controller_joint src/test_route_controller_joint.cpp)
target_include_directories(test_route_controller_joint PRIVATE include)
ament_target_dependencies(test_route_controller_joint nav2_mppi_controller nav2_core pluginlib behaviortree_cpp_v3 rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(test_route_controller_joint footprint_safe_goal_checker)
add_test(NAME route_controller_joint COMMAND test_route_controller_joint ${CMAKE_CURRENT_SOURCE_DIR}/test/data/route207.txt)
set_tests_properties(route_controller_joint PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=174" TIMEOUT 90)
set_target_properties(ordered_mppi PROPERTIES BUILD_RPATH_USE_ORIGIN TRUE)
add_custom_target(consistency_final DEPENDS test_route_controller_joint ordered_mppi)
ament_package()
