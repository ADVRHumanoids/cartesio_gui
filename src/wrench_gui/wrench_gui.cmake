
## Compile wrench widget library
add_library(wrench_gui_widget SHARED
    #src/wrench_gui/wrench_gui_widget_mainview.cpp
    src/wrench_gui/wrench_gui_widget.cpp
    src/wrench_gui/resources.qrc
    )

target_link_libraries(wrench_gui_widget PUBLIC
                        Qt5::Widgets
                        Qt5::UiTools
                        ${catkin_LIBRARIES}
                        )

set_target_properties(wrench_gui_widget PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)


install(TARGETS wrench_gui_widget
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Compile main
add_executable(wrench_gui src/wrench_gui/wrench_gui_main.cpp)

target_link_libraries(wrench_gui PRIVATE
                      wrench_gui_widget)

set_target_properties(wrench_gui PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS wrench_gui
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

