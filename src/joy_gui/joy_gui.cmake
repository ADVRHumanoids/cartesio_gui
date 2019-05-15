
find_package(catkin REQUIRED COMPONENTS roscpp cartesian_interface urdf tf roslib)
find_package(Eigen3 REQUIRED)

add_executable(joy_gui src/joy_gui/main.cpp 
                       src/joy_gui/joygui_backend.cpp 
                       src/joy_gui/viewer3d_backend.cpp
                       src/joy_gui/ui/qml.qrc)
                       
target_compile_definitions(joy_gui PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(joy_gui PRIVATE Qt5::Core Qt5::Quick ${catkin_LIBRARIES})
target_include_directories(joy_gui PRIVATE ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} src/joy_gui)
set_target_properties(joy_gui PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE) # REQUIRED TO FIND CUSTOM QT VIA RPATH ON INSTALLED EXEC!!!

install(TARGETS joy_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


get_filename_component(PATH_TO_JOY_GUI_EXEC "${CATKIN_PACKAGE_BIN_DESTINATION}/joy_gui"
    REALPATH BASE_DIR "${CMAKE_INSTALL_PREFIX}")
get_filename_component(PATH_TO_JOY_GUI_ICON "${CATKIN_PACKAGE_SHARE_DESTINATION}/console.png"
    REALPATH BASE_DIR "${CMAKE_INSTALL_PREFIX}")

configure_file(src/joy_gui/joy_gui.in.desktop joy_gui.desktop)

install(FILES src/joy_gui/pics/console.png
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/joy_gui.desktop
    DESTINATION "~/.local/share/applications")


