
include(TargetNameHelper)
GetTargetName()
add_library(${TARGET_NAME} INTERFACE)
find_package(Eigen3 REQUIRED)
target_link_libraries(${TARGET_NAME} INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp  SDL SDL_image)
target_include_directories(${TARGET_NAME} INTERFACE 
${CMAKE_CURRENT_SOURCE_DIR}
algorithm
map
point
common
)