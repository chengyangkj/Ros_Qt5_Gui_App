add_library(basic INTERFACE)

target_include_directories(basic INTERFACE 
${CMAKE_CURRENT_SOURCE_DIR}
algorithm
map
point
common
)

if(WIN32)
  find_package(SDL REQUIRED)
  find_package(SDL_image REQUIRED)
  target_link_libraries(basic INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp common SDL::SDL SDL_image::SDL_image)
else()
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(SDL REQUIRED sdl)
  pkg_check_modules(SDLIMAGE REQUIRED SDL_image)
  
  target_include_directories(basic INTERFACE ${SDL_INCLUDE_DIRS} ${SDLIMAGE_INCLUDE_DIRS})
  target_link_libraries(basic INTERFACE Eigen3::Eigen yaml-cpp::yaml-cpp common ${SDL_LIBRARIES} ${SDLIMAGE_LIBRARIES})
  target_compile_options(basic INTERFACE ${SDL_CFLAGS_OTHER} ${SDLIMAGE_CFLAGS_OTHER})
endif()