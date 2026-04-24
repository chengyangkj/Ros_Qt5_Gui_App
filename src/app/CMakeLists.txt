find_package(Qt5 COMPONENTS Widgets REQUIRED)
if(WIN32)
  add_executable(${PROJECT_NAME} WIN32 main.cpp)
else()
  add_executable(${PROJECT_NAME} main.cpp)
endif()

target_link_libraries(${PROJECT_NAME} common mainwindow framework Qt5::Core Qt5::Widgets)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
BUILD_RPATH "$ORIGIN:$ORIGIN/lib"
INSTALL_RPATH "$ORIGIN:$ORIGIN/lib"
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

if(UNIX)
  file(GLOB LINUX_BIN_FILES "${CMAKE_SOURCE_DIR}/install/linux/bin/*")
  foreach(BIN_FILE ${LINUX_BIN_FILES})
    get_filename_component(FILENAME ${BIN_FILE} NAME)
    add_custom_command(
      TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
        ${BIN_FILE}
        ${CMAKE_BINARY_DIR}/${FILENAME}
      COMMENT "Copying ${FILENAME} to build directory"
    )
    if(FILENAME MATCHES "\\.(sh|py)$")
      add_custom_command(
        TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND chmod 755
          ${CMAKE_BINARY_DIR}/${FILENAME}
        COMMENT "Setting execute permissions on ${FILENAME}"
      )
    endif()
  endforeach()
elseif(WIN32)
  file(GLOB WINDOWS_BIN_FILES "${CMAKE_SOURCE_DIR}/install/windows/bin/*")
  foreach(BIN_FILE ${WINDOWS_BIN_FILES})
    get_filename_component(FILENAME ${BIN_FILE} NAME)
    add_custom_command(
      TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy
        ${BIN_FILE}
        ${CMAKE_BINARY_DIR}/${FILENAME}
      COMMENT "Copying ${FILENAME} to build directory"
    )
  endforeach()
endif()
