set(TARGET_NAME runtime)

set(RUNTIME_SOURCES
    application_manager.cc
    plugin_manager.cc
)

set(RUNTIME_HEADERS
    application_manager.h
    plugin_manager.h
)

add_library(${TARGET_NAME} STATIC
    ${RUNTIME_SOURCES}
    ${RUNTIME_HEADERS}
)

target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(${TARGET_NAME} PUBLIC mainwindow common)

set_target_properties(${TARGET_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
)

install(TARGETS ${TARGET_NAME}
    RUNTIME DESTINATION bin/lib
    LIBRARY DESTINATION bin/lib
    ARCHIVE DESTINATION bin/lib
)

