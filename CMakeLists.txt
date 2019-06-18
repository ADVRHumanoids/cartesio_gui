cmake_minimum_required(VERSION 3.1.0)
project(cartesio_gui)

add_compile_options(-std=c++14)
add_definitions(-DXBOT_MSGS_SUPPORT)
option(QT_VERSION_GREATER_THAN_5_5 "Set to true if Qt5 > 5.5 is installed" FALSE)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

if(${QT_VERSION_GREATER_THAN_5_5})
    set(MINIMUM_QT_VERSION 5.9)
else()
    set(MINIMUM_QT_VERSION 5.5)
endif()

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
find_package(catkin REQUIRED COMPONENTS cartesian_interface rviz tf roslib)
find_package(XBotInterface REQUIRED)

catkin_package()

include_directories(include/
                    ${catkin_INCLUDE_DIRS})

include(src/qt_utils/qt_utils.cmake)
include(src/cartesio/cartesio.cmake)
include(src/sliders/sliders.cmake)
include(src/wrench_gui/wrench_gui.cmake)
include(src/cartesian_impedance_gui/cartesian_impedance_gui.cmake)

if(${QT_VERSION_GREATER_THAN_5_5})
    include(src/joy_gui/joy_gui.cmake)
endif()

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
