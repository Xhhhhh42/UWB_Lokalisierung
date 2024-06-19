cmake_minimum_required(VERSION 3.12.0)
project(uwb_signal_processing)

add_definitions(-std=c++14 -Wall -Wno-sign-compare -Wno-catch-value -Wno-switch -Wno-maybe-uninitialized -Werror=return-type )

find_package(catkin REQUIRED COMPONENTS
  tf2
  tf2_ros
)
find_package(catkin_simple REQUIRED)

catkin_simple(ALL_DEPS_REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/uwb_filter.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )


############
# BINARIES #
############
cs_add_executable(uwb_filter_node
  src/uwb_filter_node.cpp
)
target_link_libraries(uwb_filter_node ${PROJECT_NAME} )


cs_install()
cs_export()
