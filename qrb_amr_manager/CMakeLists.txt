cmake_minimum_required(VERSION 3.8)
project(qrb_amr_manager)

set(library_name qrb_amr_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${library_name} SHARED
  src/amr_manager.cpp
  src/amr_state_machine.cpp
  src/low_power_manager.cpp
)

aux_source_directory(./src SRC_LIST)

ament_target_dependencies(qrb_amr_manager
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
)

install(TARGETS qrb_amr_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)


file(GLOB_RECURSE HEADER_FILES
  "${CMAKE_SOURCE_DIR}/include/*.h"
  "${CMAKE_SOURCE_DIR}/include/*.hpp"
)

install(FILES ${HEADER_FILES}
  DESTINATION include/${PROJECT_NAME}/
)

install(DIRECTORY
  include
  DESTINATION include/${PROJECT_NAME}/
)

ament_package()
