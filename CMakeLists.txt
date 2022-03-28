cmake_minimum_required(VERSION 2.8.3)

project(pva_tracker)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	tf
	dynamic_reconfigure
	)

generate_dynamic_reconfigure_options(
		cfg/pid.cfg
		cfg/pid_ground.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  "../../devel/include"
  "include"
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_definitions(${EIGEN_DEFINITIONS})

find_package(roslaunch)


add_executable(tracker src/tracker.cpp)
target_link_libraries(tracker ${catkin_LIBRARIES})
add_dependencies(tracker ${PROJECT_NAME}_gencfg)

add_executable(tracker_enu src/tracker_enu.cpp)
target_link_libraries(tracker_enu ${catkin_LIBRARIES})
add_dependencies(tracker_enu ${PROJECT_NAME}_gencfg)

add_executable(tracker_sim_auto_arm_takeoff_enu src/tracker_sim_auto_arm_takeoff_enu.cpp)
target_link_libraries(tracker_sim_auto_arm_takeoff_enu ${catkin_LIBRARIES})
add_dependencies(tracker_sim_auto_arm_takeoff_enu ${PROJECT_NAME}_gencfg)

add_executable(control_test src/control_test.cpp)
target_link_libraries(control_test ${catkin_LIBRARIES})

add_executable(hover_test src/hover_test.cpp)
target_link_libraries(hover_test ${catkin_LIBRARIES})






