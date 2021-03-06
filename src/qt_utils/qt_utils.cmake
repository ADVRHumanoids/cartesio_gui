
## Compile wrench widget library
add_library(qt_utils SHARED
    src/qt_utils/qt_utils.cpp
    src/qt_utils/resources.qrc
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

## Compile main
add_executable(test_main src/qt_utils/test_main.cpp)

target_link_libraries(test_main PRIVATE
                      qt_utils)

set_target_properties(test_main PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

install(TARGETS test_main
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
