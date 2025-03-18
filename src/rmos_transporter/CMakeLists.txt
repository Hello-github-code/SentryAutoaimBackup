cmake_minimum_required(VERSION 3.8)
project(rmos_transporter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE -Ofast)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DPATH=\"\\\"${PROJECT_SOURCE_DIR}\\\"\"")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D${CMAKE_SYSTEM_NAME}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

set(INCLUDES ${INCLUDES} $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set(INCLUDES ${INCLUDES} $<INSTALL_INTERFACE:/include>)

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/src/Algorithm/include
  ${3RDPARTY_LIB}
  ${EIGEN3_INCLUDE_DIR}
  /usr/include/eigen3
  ${OpenCV_INCLUDE_DIR}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY
#   DESTINATION share/${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rmos_transporter::UsbCommNode
  EXECUTABLE usb_comm
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
