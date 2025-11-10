add_subdirectory(framework)
add_subdirectory(runtime)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
add_executable(
    ${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}  common mainwindow framework runtime Qt5::Core Qt5::Widgets)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
BUILD_RPATH "./lib"
INSTALL_RPATH "./lib"
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)

# 在构建时将启动脚本复制到可执行程序同级别目录（build 目录）
if(UNIX)
  # Linux 系统：复制 linux/bin 目录下的所有文件
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
    # 如果是脚本文件，设置可执行权限
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
  # Windows 系统：复制 windows/bin 目录下的所有文件
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