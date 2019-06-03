cmake_minimum_required(VERSION 2.8.3)
project(bno055_uart_driver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        serial
)

catkin_package(
    CATKIN_DEPENDS sensor_msgs
)

include_directories(
        ${Boost_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_executable(bno055_uart_driver_node src/BNO055UARTDriver.cpp src/BNO055UARTDriverNode.cpp src/bno055.c src/bno055_uart.cpp)
target_link_libraries(bno055_uart_driver_node ${Boost_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(bno055_uart_driver_node ${catkin_EXPORTED_TARGETS})


install(TARGETS bno055_uart_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
