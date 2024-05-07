cmake_minimum_required(VERSION 2.8.3)
project(VHDMap-SE)
set(CMAKE_CXX_FLAGS "-std=c++14")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PROJ REQUIRED)
find_package(absl REQUIRED)


#add_message_files(
#   FILES
# )

generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES VHDMap-SE
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs
  #DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJ_INCLUDE_DIR}
)

list(APPEND SOURCE_FILES
    ${PROJECT_SOURCE_DIR}/src/main.cc
    ${PROJECT_SOURCE_DIR}/src/common/node_parameters.cc
    ${PROJECT_SOURCE_DIR}/src/common/system_parameters.cc
    ${PROJECT_SOURCE_DIR}/src/lib/error_state_kalman_filter.cc
    ${PROJECT_SOURCE_DIR}/src/lib/navigation_system.cc
    ${PROJECT_SOURCE_DIR}/src/lib/parallel_filters.cc
    ${PROJECT_SOURCE_DIR}/src/lib/sensor_data.cc
    ${PROJECT_SOURCE_DIR}/src/lib/navigation_node.cc
    ${PROJECT_SOURCE_DIR}/src/lib/mercator.cc
)

list(APPEND LINK_LIBS 
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    ${PROJ_LIBRARIES}
    proj
    absl::strings
)

add_executable(navigation_node ${SOURCE_FILES})
#add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation_node ${LINK_LIBS})

# Executable file install
install(TARGETS navigation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#Folder installation
install(DIRECTORY launch
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include
         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY conf
         DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
