cmake_minimum_required(VERSION 3.10)
project(mujoco-3.2.0)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")
enable_language(C)
enable_language(CXX)
set(CMAKE_BUILD_TYPE Release)

set(pinocchio_DIR "/opt/openrobots/lib/cmake/pinocchio/")
set(MUJOCO_PATH "/home/jaehun/mujoco-3.2.0/")
set(SIMULATE_PATH "${MUJOCO_PATH}/simulate")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

## System dependencies are found with CMake's conventions
find_package(Curses REQUIRED)
find_package(glfw3 REQUIRED)
find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(pinocchio REQUIRED)


###########
## Build ##
###########

find_library(MUJOCO
  NAMES libmujoco.so.3.2.0
  PATHS ${MUJOCO_PATH}/lib
  REQUIRED
)

add_library(lodepng STATIC ${SIMULATE_PATH}/lodepng.h ${SIMULATE_PATH}/lodepng.cpp)
add_library(libsimulate STATIC)
add_library(control STATIC include/Controller.cpp)

target_sources(
  libsimulate
  PUBLIC  ${SIMULATE_PATH}/simulate.h
  PRIVATE ${SIMULATE_PATH}/simulate.cc
          ${SIMULATE_PATH}/array_safety.h
          ${SIMULATE_PATH}/glfw_adapter.h
          ${SIMULATE_PATH}/glfw_adapter.cc
          ${SIMULATE_PATH}/platform_ui_adapter.h
          ${SIMULATE_PATH}/platform_ui_adapter.cc
          ${SIMULATE_PATH}/glfw_dispatch.h
          ${SIMULATE_PATH}/glfw_dispatch.cc
          
)
target_link_libraries(libsimulate PUBLIC lodepng ${MUJOCO})
target_link_options(libsimulate PRIVATE ${MUJOCO_SIMULATE_LINK_OPTIONS})
target_compile_options(libsimulate PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
target_include_directories(libsimulate 
  PUBLIC 
    ${SIMULATE_PATH}
    ${MUJOCO_PATH}/include
    ${MUJOCO_PATH}/simulate
    ${MUJOCO_PATH}/sample
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${MUJOCO_PATH}/include/mujoco
  ${MUJOCO_PATH}/include
  ${MUJOCO_PATH}/simulate
  ${MUJOCO_PATH}/sample
  ${GLFW_INCLUDE_DIR}
  ${CURSES_INCLUDE_DIR}
)

## Declare a C++ library

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}
  simulate/main.cc
  ${SIMULATE_PATH}/array_safety.h
  ${SIMULATE_PATH}/glfw_dispatch.cc
  ${SIMULATE_PATH}/glfw_dispatch.h
  ${SIMULATE_PATH}/glfw_adapter.cc
  ${SIMULATE_PATH}/glfw_adapter.h
  ${SIMULATE_PATH}/platform_ui_adapter.cc
  ${SIMULATE_PATH}/platform_ui_adapter.h
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(control Eigen3::Eigen pinocchio::pinocchio ${MUJOCO})

target_link_libraries(${PROJECT_NAME}
  lodepng
  Eigen3::Eigen
  pinocchio::pinocchio
  glfw 
  ${GLFW3_LIBRARY} 
  libsimulate
  ${CURSES_LIBRARY}
  -lpthread 
  ${MUJOCO}
  control
) 


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mujoco_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
