cmake_minimum_required(VERSION 2.8.3)
project(nostop_agent_sensor)

ENABLE_LANGUAGE(CXX)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy
  std_msgs 
  base_math
  serial
  )

# find_package( PCL 1.7.1 REQUIRED) # COMPONENT common io visualization)
# 
# include_directories(BEFORE ${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})




################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
 #add_message_files(
 #  FILES
#    AgentSensorData.msg
#    SensorData.msg
     #Id_robot.msg
     #Camera_data.msg
# )

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
   std_msgs  # Or other packages containing msgs
   )

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES ${PROJECT_NAME}
  DEPENDS roscpp rospy base_math 
)



###########
## Build ##
###########
include_directories(include/${PROJECT_NAME}/ ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
include_directories(src/)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/src)
include_directories(${CMAKE_INSTALL_PREFIX}/include) # this line is to find lemon include files.
include_directories(${OpenCV_INCLUDE_DIRS})

set(agent_sensor_SOURCES
        src/agent_sensor.cpp
        src/Sensor_reader.cpp
)

add_executable(agent_sensor ${agent_sensor_SOURCES})
add_dependencies(agent_sensor nostop_agent_sensor_gencpp)
target_link_libraries(agent_sensor ${catkin_LIBRARIES} ${OpenCV_LIBS} base_math lemon)#${PCL_LIBRARIES})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
 install(TARGETS agent_sensor
 EXPORT nostop_agent_sensor
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
