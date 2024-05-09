cmake_minimum_required(VERSION 3.8)
project(bt_avoidance)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 20)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(bt_onward_node SHARED src/bt_avoidance/Onward.cpp)
add_library(bt_move_away_node SHARED src/bt_avoidance/MoveAway.cpp)
add_library(bt_any_obstacle_node SHARED src/bt_avoidance/AnyObstacle.cpp)

list(APPEND plugin_libs
  bt_onward_node
  bt_move_away_node
  bt_any_obstacle_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_avoidance src/bt_avoidance.cpp)
ament_target_dependencies(bt_avoidance ${dependencies})
target_link_libraries(bt_avoidance ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  bt_avoidance
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY bt_xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
