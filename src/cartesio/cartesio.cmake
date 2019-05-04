## Compile Cartesio RViz Panel
add_library(cartesio_rviz_panel SHARED 
    include/cartesio_gui/cartesio/task_options_widget.h
    src/cartesio/cartesio_rviz_panel.cpp
    src/cartesio/task_options_widget.cpp)

target_link_libraries(cartesio_rviz_panel Qt5::Widgets ${catkin_LIBRARIES})

install(TARGETS cartesio_rviz_panel
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  cartesio_rviz_panel_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
