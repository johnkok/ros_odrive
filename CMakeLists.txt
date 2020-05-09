cmake_minimum_required(VERSION 2.8.3)
project(ros_odrive)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
  FILES
  odrive_msg.msg
  odrive_ctrl.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

set(rt_LIBRARIES rt)
set(pthread_LIBRARIES pthread)
set(usb-1.0_LIBRARIES usb-1.0)
set(jsoncpp_LIBRARIES jsoncpp)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
  DEPENDS ${rt_LIBRARIES} ${pthread_LIBRARIES} ${usb-1.0_LIBRARIES} ${jsoncpp_LIBRARIES}
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ros_odrive src/odrive.cpp src/odrive_endpoint.cpp src/odrive_utils.cpp)
add_dependencies(ros_odrive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_odrive ${usb-1.0_LIBRARIES} ${catkin_LIBRARIES} ${jsoncpp_LIBRARIES})

#############
## Install ##
#############

install(TARGETS ros_odrive 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ros_odrive
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

