cmake_minimum_required(VERSION 3.5)

set(EXAMPLES_BIN_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin)
set(EXAMPLES_LIB_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/lib)

if($ENV{ROS_VERSION} EQUAL 1)
  set(EXAMPLES_BIN_INSTALL_PREFIX ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(EXAMPLES_LIB_INSTALL_PREFIX ${CATKIN_GLOBAL_LIB_DESTINATION})
elseif($ENV{ROS_VERSION} EQUAL 2)
  set(EXAMPLES_BIN_INSTALL_PREFIX lib/${PROJECT_NAME})
  set(EXAMPLES_LIB_INSTALL_PREFIX lib)
endif()

find_package(Eigen3 REQUIRED)

include_directories(./)
include_directories(${EIGEN3_INCLUDE_DIR})

set(COMMON_SRCS
  ./pf_controller_base.cpp
)

if (WIN32)
  set(LINK_LIBS pointfoot_sdk_lowlevel shlwapi)
else()
  set(LINK_LIBS pointfoot_sdk_lowlevel pthread)
endif()

add_executable(pf_joint_move pf_joint_move.cpp ${COMMON_SRCS})
target_link_libraries(pf_joint_move ${LINK_LIBS})
install(TARGETS pf_joint_move DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

add_executable(pf_groupJoints_move pf_groupJoints_move.cpp ${COMMON_SRCS})
target_link_libraries(pf_groupJoints_move ${LINK_LIBS})
install(TARGETS pf_groupJoints_move DESTINATION ${EXAMPLES_BIN_INSTALL_PREFIX})

