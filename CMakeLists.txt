cmake_minimum_required(VERSION 2.8.3)
project(pas_de_deux)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${QT_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
                  Color.msg Pose.msg)
add_service_files(DIRECTORY srv FILES
                  Spawn.srv
                  Sensor.srv)
generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

set(CMAKE_CXX_FLAGS "-lpigpio -lrt ${CMAKE_CXX_FLAGS}")

set(turtlesim_node_SRCS
  src/turtle.cpp
  src/animate.cpp
  src/agent.cpp
)
set(turtlesim_node_HDRS
  include/turtlesim/animate.h
)

qt4_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

add_executable(srmrvr_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(srmrvr_node ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(srmrvr_node turtlesim_gencpp)

add_executable(animate ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(animate ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(animate turtlesim_gencpp)

add_executable(swarm_node src/pasdedeux.cpp)
target_link_libraries(swarm_node ${catkin_LIBRARIES} -lwiringPi)

install(TARGETS animate swarm_node srmrvr_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
