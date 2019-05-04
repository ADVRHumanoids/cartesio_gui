cmake_minimum_required(VERSION 3.1.0)
project(cartesio_gui)

add_compile_options(-std=c++14)
add_definitions(-DXBOT_MSGS_SUPPORT)

set(CMAKE_AUTORCC ON)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5UiTools CONFIG REQUIRED)
find_package(catkin REQUIRED COMPONENTS cartesian_interface rviz)
find_package(XBotInterface REQUIRED)

catkin_package()

include_directories(include/ ${Qt5Widgets_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

include(src/cartesio/cartesio.cmake)
include(src/sliders/sliders.cmake)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
