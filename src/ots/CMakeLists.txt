set(SOURCES
    BasicGtuLifecycleController.cc
    Core.cc
    GtuObject.cc
    RadioMessage.cc
    UniformGtuCreationPolicy.cc
    RosNode.cc
)

add_library(ots SHARED ${SOURCES})
target_include_directories(ots PUBLIC $<TARGET_PROPERTY:core,INCLUDE_DIRECTORIES>)
set_property(TARGET ots PROPERTY NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
set_property(TARGET ots PROPERTY OMNETPP_LIBRARY ON)

macro(find_ros)
    set(ROS_INCLUDE_DIRS "")
    set(ROS_LIBRARIES "")
    foreach(ros_pkg ${ARGV})
        find_package("${ros_pkg}" REQUIRED)
        list(APPEND ROS_INCLUDE_DIRS "${${ros_pkg}_INCLUDE_DIRS}")
        list(APPEND ROS_LIBRARIES "${${ros_pkg}_LIBRARIES}")
    endforeach()
endmacro()

find_ros(rclcpp gazebo_msgs)
target_include_directories(ots PUBLIC ${ROS_INCLUDE_DIRS})
target_link_libraries(ots PUBLIC ${ROS_LIBRARIES})

find_package(PkgConfig MODULE REQUIRED)
pkg_check_modules(ZEROMQ REQUIRED libzmq)
target_include_directories(ots PUBLIC "${ZEROMQ_INCLUDE_DIRS}")
target_link_libraries(ots PUBLIC "${ZEROMQ_LIBRARIES}")

set(SIM0MQPP_INSTALL ON)
set(SIM0MQPP_SHARED_LIB ON)
add_subdirectory(${PROJECT_SOURCE_DIR}/extern/sim0mqpp ${PROJECT_BINARY_DIR}/sim0mqpp)
target_link_libraries(ots PUBLIC sim0mqpp::sim0mqpp)

install(TARGETS ots LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${CMAKE_INSTALL_DATADIR}/ned/ots FILES_MATCHING PATTERN "*.ned")
set_property(TARGET ots APPEND PROPERTY INSTALL_NED_FOLDERS ${CMAKE_INSTALL_DATADIR}/ned/ots)
