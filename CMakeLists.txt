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


add_executable(tracker_auto_takeoff_optitrack src/tracker_auto_takeoff_optitrack.cpp)
target_link_libraries(tracker_auto_takeoff_optitrack ${catkin_LIBRARIES})
add_dependencies(tracker_auto_takeoff_optitrack ${PROJECT_NAME}_gencfg)


add_executable(tracker_sim_auto_arm_takeoff src/tracker_sim_auto_arm_takeoff.cpp)
target_link_libraries(tracker_sim_auto_arm_takeoff ${catkin_LIBRARIES})
add_dependencies(tracker_sim_auto_arm_takeoff ${PROJECT_NAME}_gencfg)


add_executable(sim_control_test src/sim_control_test.cpp)
target_link_libraries(sim_control_test ${catkin_LIBRARIES})

add_executable(real_control_test_optimal_jerk src/real_control_test_optimal_jerk.cpp)
target_link_libraries(real_control_test_optimal_jerk ${catkin_LIBRARIES})


add_executable(sim_hover_test src/sim_hover_test.cpp)
target_link_libraries(sim_hover_test ${catkin_LIBRARIES})

add_executable(fake_bebop_odom_generator src/fake_bebop_odom_generator.cpp)
target_link_libraries(fake_bebop_odom_generator ${catkin_LIBRARIES})




