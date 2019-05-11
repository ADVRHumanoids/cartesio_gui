cmake_minimum_required(VERSION 3.1.0)
project(cartesio_gui)

add_compile_options(-std=c++14)
add_definitions(-DXBOT_MSGS_SUPPORT)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(MINIMUM_QT_VERSION 5.9)

## Get custom path to QT from env
set(QT_PATH $ENV{CARTESIO_GUI_QT_PATH})
message(STATUS "Searching QT in '${QT_PATH}'")

# Get CMAKE_PREFIX_PATH from environment, and append QT_PATH
set(CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
string(REPLACE ":" ";" CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH})
list(APPEND CMAKE_PREFIX_PATH ${QT_PATH})

find_package(Qt5UiTools ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Widgets ${MINIMUM_QT_VERSION} REQUIRED)
find_package(Qt5Quick ${MINIMUM_QT_VERSION} REQUIRED)
find_package(catkin REQUIRED COMPONENTS cartesian_interface rviz)
find_package(XBotInterface REQUIRED)

catkin_package()

include_directories(include/
                    ${catkin_INCLUDE_DIRS})

include(src/cartesio/cartesio.cmake)
include(src/sliders/sliders.cmake)
include(src/joy_gui/joy_gui.cmake)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
