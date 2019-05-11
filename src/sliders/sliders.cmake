 
## Compile slider widget library
add_library(sliders_widget SHARED 
    src/sliders/sliders_widget_mainview.cpp
    src/sliders/sliders_widget.cpp
    src/sliders/resources.qrc
    )
    
target_link_libraries(sliders_widget PUBLIC 
                        Qt5::Widgets 
                        Qt5::UiTools
                        ${catkin_LIBRARIES}
                        XBotInterface::XBotInterface)
                         
install(TARGETS sliders_widget
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Compile main
add_executable(joint_state_sliders src/sliders/sliders_widget_main.cpp)
                                
target_link_libraries(joint_state_sliders PRIVATE 
                        sliders_widget)

install(TARGETS joint_state_sliders
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Compile Sliders RViz Panel
add_library(sliders_rviz_panel SHARED 
    src/sliders/sliders_rviz_panel.cpp
)

target_link_libraries(sliders_rviz_panel PRIVATE sliders_widget)

install(TARGETS sliders_rviz_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  sliders_rviz_panel_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

