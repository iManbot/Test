cmake_minimum_required(VERSION 2.8.3)
project(robot_controller)

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

# uncomment this in final release
set(CMAKE_BUILD_TYPE Debug)
#set(DCMAKE_BUILD_TYPE:=Debug)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files( 
   DIRECTORY
   msg
   FILES
   SpeedUVW.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_controller
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_BINARY_DIR}/src/wrappers/matrix
  ${CMAKE_SOURCE_DIR}/src
  ${RNG_INCLUDE}
  ${MATRIX_INCLUDE}
)

find_package(PkgConfig)

pkg_check_modules(BFL REQUIRED bfl)

message("BFL include dirs:" ${BFL_INCLUDE_DIRS})
message("BFL library dirs:" ${BFL_LIBRARY_DIRS})

include_directories(${BFL_INCLUDE_DIRS})
link_directories(${BFL_LIBRARY_DIRS})

## Declare a cpp library
# add_library(robot_controller
#   src/${PROJECT_NAME}/robot_controller.cpp
# )
add_library(robot_controller
   src/KinematicModel.cpp src/BaseKinematicModel.cpp
   src/ArmKinematicModel.cpp
   src/Controller.cpp src/PController.cpp
   src/DummyController.cpp
   src/Transmission.cpp
 )


## Declare a cpp executable
# add_executable(robot_controller_node src/robot_controller_node.cpp)
#add_executable(imb_kinect src/imb_kinect.cpp)
#add_executable(imb_gestures src/imb_gestures.cpp)
add_executable(robot src/Robot.cpp)

add_executable(base_driver src/BaseDriver.cpp)
add_executable(base_driverUSB src/BaseDriverUSB.cpp)
add_executable(arm_driverUSB src/ArmDriverUSB.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_controller_node robot_controller_generate_messages_cpp)
#add_dependencies(robot robot_generate_messages_cpp)
#add_dependencies(imb_gestures imb_kinect_generate_messages_cpp)
#add_dependencies(imb_arm_ctrl imb_arm_ctrl_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(robot_controller_node
#   ${catkin_LIBRARIES}
# )

catkin_install_python(PROGRAMS
   src/teleop_vector3_keyboard.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

target_link_libraries(robot
   ${catkin_LIBRARIES} robot_controller ${BFL_LIBRARIES}
 )

 target_link_libraries(base_driver
   ${catkin_LIBRARIES} -lbluetooth
 )

 target_link_libraries(base_driverUSB
   ${catkin_LIBRARIES}
 )
 
 target_link_libraries(arm_driverUSB
   ${catkin_LIBRARIES}
 )
#  target_link_libraries(imb_gestures
#   ${catkin_LIBRARIES}
# )
 
#   target_link_libraries(imb_arm_ctrl
#   ${catkin_LIBRARIES}
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS robot_controller robot_controller_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_robot_controller.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
