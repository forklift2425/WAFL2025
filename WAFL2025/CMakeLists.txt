cmake_minimum_required(VERSION 2.8.3)

project(WAFL2025)

find_package(catkin REQUIRED COMPONENTS
 message_generation
 roscpp
 rospy
 std_msgs
 geometry_msgs
 urdf
 xacro
)

add_message_files(
  FILES
  WheelSpeeds.msg
  SteeringAngles.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
	geometry_msgs
	roscpp
	rospy
	std_msgs
	message_runtime
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
