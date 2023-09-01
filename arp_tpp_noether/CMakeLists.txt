cmake_minimum_required(VERSION 3.5)
project(arp_tpp_noether)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


##################
## Dependencies ##
##################

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(noether_tpp REQUIRED)

find_package(boost_plugin_loader REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(arp_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)


##########################
## Compile Instructions ##
##########################

add_executable(${PROJECT_NAME}_node src/arp_tpp_noether_node.cpp)
ament_target_dependencies(
    ${PROJECT_NAME}_node
    rclcpp
    geometry_msgs
    arp_msgs
    tf2_eigen)
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} noether::noether_tpp)


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME})


# Misc Pkg Folders
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
