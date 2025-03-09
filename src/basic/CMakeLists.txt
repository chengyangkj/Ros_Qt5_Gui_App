add_library(basic INTERFACE)

target_link_libraries(basic INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp  SDL SDL_image)

target_include_directories(basic INTERFACE 
${CMAKE_CURRENT_SOURCE_DIR}
algorithm
map
point
common
)