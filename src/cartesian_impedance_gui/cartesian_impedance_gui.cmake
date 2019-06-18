add_executable(cartesian_impedance_gui src/cartesian_impedance_gui/cartesian_impedance_gui_main.cpp
				       src/cartesian_impedance_gui/resources.qrc)

find_package(cartesian_interface REQUIRED)
target_link_libraries(cartesian_impedance_gui PRIVATE
                        Qt5::Widgets
                        Qt5::UiTools
                    ${catkin_LIBRARIES}
                      qt_utils
                      ${cartesian_interface_LIBRARIES})
include_directories(${cartesian_interface_INCLUDES})

set_target_properties(cartesian_impedance_gui PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS cartesian_impedance_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

