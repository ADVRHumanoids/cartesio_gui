
## Compile wrench widget library
add_library(wrench_gui_widget SHARED
    #src/wrench_gui/wrench_gui_widget_mainview.cpp
    src/wrench_gui/wrench_gui_widget.cpp
    src/wrench_gui/wrench_gui_resources.qrc
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

## Compile Sliders RQT Panel
add_library(wrench_rqt SHARED 
    src/wrench_gui/wrench_rqt.cpp
)

target_link_libraries(wrench_rqt PRIVATE wrench_gui_widget)

install(TARGETS wrench_rqt
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  wrench_rqt_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

