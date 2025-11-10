# 安装启动脚本（用于 make install）
if(UNIX)
  # Linux 系统：安装 linux/bin 目录下的所有文件
  file(GLOB LINUX_BIN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/linux/bin/*")
  install(FILES ${LINUX_BIN_FILES}
    DESTINATION bin
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
  )
elseif(WIN32)
  # Windows 系统：安装 windows/bin 目录下的所有文件
  file(GLOB WINDOWS_BIN_FILES "${CMAKE_CURRENT_SOURCE_DIR}/windows/bin/*")
  install(FILES ${WINDOWS_BIN_FILES}
    DESTINATION bin
  )
endif()

