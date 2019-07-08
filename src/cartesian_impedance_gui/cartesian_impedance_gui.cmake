add_library(cartesian_impedance_gui SHARED src/cartesian_impedance_gui/cartesian_impedance_gui.cpp
                                           src/cartesian_impedance_gui/cartesian_impedance_gui_resources.qrc)

find_package(cartesian_interface REQUIRED)
target_link_libraries(cartesian_impedance_gui PUBLIC
                        qt_utils
                        Qt5::Widgets
                        Qt5::UiTools
                      ${catkin_LIBRARIES}
                      ${cartesian_interface_LIBRARIES})

include_directories(${cartesian_interface_INCLUDES})

set_target_properties(cartesian_impedance_gui PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS cartesian_impedance_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Compile Sliders RQT Panel
add_library(cartesian_impedance_gui_rqt SHARED
    src/cartesian_impedance_gui/cartesian_impedance_gui_rqt.cpp
)

target_link_libraries(cartesian_impedance_gui_rqt PRIVATE cartesian_impedance_gui)

install(TARGETS cartesian_impedance_gui_rqt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  cartesian_impedance_gui_rqt_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
