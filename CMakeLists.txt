cmake_minimum_required(VERSION 2.8.3)
project(pocketsphinx_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
  bender_srvs
  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED COMPONENTS system signals thread)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html



catkin_python_setup()


################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   RecognitionOrder.srv
 )


add_action_files(
   DIRECTORY 
   action
   FILES
   DoRecognition.action
)


 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed


 generate_dynamic_reconfigure_options(
   cfg/SpeechRecognition.cfg
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
#  LIBRARIES pocketsphinx_ros
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/sphinxbase/
  /usr/local/include/pocketsphinx/
)

link_directories(
  /usr/local/lib
)


## - - - recognizer - - - 
add_executable(recognizer src/recognizer.cpp)
add_dependencies(recognizer ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognizer ${catkin_LIBRARIES} ${Boost_LIBRARIES} pocketsphinx sphinxbase sphinxad pulse pulse-simple)


## - - - recognizeralfa - - - 
add_executable(recognizeralfa src/recognizeralfa.cpp)
add_dependencies(recognizeralfa ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognizeralfa ${catkin_LIBRARIES} ${Boost_LIBRARIES} pocketsphinx sphinxbase sphinxad pulse pulse-simple)

## - - - recognizergamma- - - 
add_executable(recognizergamma src/recognizergamma.cpp)
add_dependencies(recognizergamma ${catkin_EXPORTED_TARGETS})
target_link_libraries(recognizergamma ${catkin_LIBRARIES} ${Boost_LIBRARIES} pocketsphinx sphinxbase sphinxad pulse pulse-simple)

## - - - tester - - - 
add_executable(tester src/tester.cpp)
add_dependencies(
  tester
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries( 
  tester
  ${catkin_LIBRARIES}
)






