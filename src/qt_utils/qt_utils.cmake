
## Compile wrench widget library
add_library(qt_utils SHARED
    src/qt_utils/qt_utils.cpp
    )

target_link_libraries(qt_utils PUBLIC
                        Qt5::Widgets
                        Qt5::UiTools
                        ${catkin_LIBRARIES}
                        )

set_target_properties(qt_utils PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)


install(TARGETS qt_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


