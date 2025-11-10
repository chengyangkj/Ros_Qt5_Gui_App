add_subdirectory(framework)
add_subdirectory(runtime)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
add_executable(
    ${PROJECT_NAME} main.cpp)

target_link_libraries(${PROJECT_NAME}  common mainwindow framework runtime Qt5::Core Qt5::Widgets)
set_target_properties(${PROJECT_NAME} PROPERTIES
RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}"
LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
BUILD_RPATH "$ORIGIN:$ORIGIN/lib"
INSTALL_RPATH "$ORIGIN:$ORIGIN/lib"
)

install(TARGETS ${PROJECT_NAME}
        RUNTIME DESTINATION bin)