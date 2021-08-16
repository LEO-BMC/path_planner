cmake_minimum_required(VERSION 2.8.3)
project(hybrid_astar)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-g3 -O3")
find_package(gflags REQUIRED)
find_package(folly REQUIRED)
find_package(yaml-cpp)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        tf
        tf2_eigen
        grid_map_core
        grid_map_ros
        grid_map_cv
        grid_map_filters
        grid_map_loader
        grid_map_msgs
        grid_map_octomap
        grid_map_rviz_plugin
        grid_map_visualization
        )

set(SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/algorithm.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/node2d.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/node3d.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/collisiondetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/planner.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/path.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/smoother.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/visualize.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dubins.cpp #Andrew Walker
        ${CMAKE_CURRENT_SOURCE_DIR}/src/dynamicvoronoi.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
        ${CMAKE_CURRENT_SOURCE_DIR}/src/bucketedqueue.cpp #Boris Lau, Christoph Sprunk, Wolfram Burgard
        )
set(HEADERS
        ${CMAKE_CURRENT_SOURCE_DIR}/include/algorithm.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/node2d.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/node3d.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/collisiondetection.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/planner.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/path.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/smoother.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/vector2d.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/visualize.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/helper.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/constants.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/lookup.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/gradient.h #Andrew Noske
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dubins.h #Andrew Walker
        ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamicvoronoi.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
        ${CMAKE_CURRENT_SOURCE_DIR}/include/bucketedqueue.h #Boris Lau, Christoph Sprunk, Wolfram Burgard
        ${CMAKE_CURRENT_SOURCE_DIR}/include/point.h #Boris Lau, Christoph Sprunk, Wolfram Burgard

        )
add_library(HYAS ${SOURCES} ${HEADERS})

## Declare a catkin package
catkin_package(CATKIN_DEPENDS
        roscpp
        rospy
        std_msgs
        tf
        tf2_eigen
        )

## OPEN MOTION PLANNING LIBRARY
find_package(ompl REQUIRED)

if (NOT OMPL_FOUND)
    message(AUTHOR_WARNING,"Open Motion Planning Library not found")
endif (NOT OMPL_FOUND)

include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${OMPL_INCLUDE_DIRS}
        ${folly_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIRS})

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
target_link_libraries(tf_broadcaster ${catkin_LIBRARIES})

add_executable(hybrid_astar src/main.cpp ${HEADERS} ${SOURCES})
target_link_libraries(hybrid_astar ${catkin_LIBRARIES}
        ${OMPL_LIBRARIES}
        Folly::folly
        gflags
        ${YAML_CPP_LIBRARIES})

install(TARGETS ${PROJECT_NAME} tf_broadcaster
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
