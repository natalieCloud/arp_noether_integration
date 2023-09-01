cmake_minimum_required(VERSION 3.5)
project(arp_msgs)

####################
## Compiler Setup ##
####################
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
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


#############################
## Generate Custom Msg/Srv ##
#############################
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ToolPath.msg"
  "srv/GenerateToolPath.srv"
  DEPENDENCIES
  geometry_msgs
  )


#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


###################
## Ament package ##
###################
ament_export_dependencies(geometry_msgs rosidl_default_runtime)
ament_package()