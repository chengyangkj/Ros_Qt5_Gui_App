add_library(basic INTERFACE)

target_include_directories(basic INTERFACE 
${CMAKE_CURRENT_SOURCE_DIR}
algorithm
map
point
common
)

if(WIN32)
  find_package(SDL2 REQUIRED)
  find_package(SDL2_image REQUIRED)
  target_link_libraries(basic INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp common SDL2::SDL2 SDL2_image::SDL2_image)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SDL2 REQUIRED sdl2)
  pkg_check_modules(SDL2IMAGE REQUIRED SDL2_image)
  
  target_include_directories(basic INTERFACE ${SDL2_INCLUDE_DIRS} ${SDL2IMAGE_INCLUDE_DIRS})
  target_link_libraries(basic INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp common ${SDL2_LIBRARIES} ${SDL2IMAGE_LIBRARIES})
  target_compile_options(basic INTERFACE ${SDL2_CFLAGS_OTHER} ${SDL2IMAGE_CFLAGS_OTHER})
endif()