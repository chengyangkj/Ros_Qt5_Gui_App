add_executable(benchmark 
    benchmark.cpp
    glaze_benchmark.cpp
    include/simdjson/simdjson.cpp
    )
target_compile_definitions(benchmark PRIVATE CATCH_CONFIG_ENABLE_BENCHMARKING)
target_include_directories(benchmark PRIVATE ${PROJECT_SOURCE_DIR}/include)
if (MSVC)
  target_compile_options(benchmark PRIVATE /w)
else()
  target_compile_options(benchmark PRIVATE -w)
endif()

add_library(catch_main_benchmark STATIC catch-main.cpp)
target_compile_definitions(catch_main_benchmark PRIVATE CATCH_CONFIG_ENABLE_BENCHMARKING)
target_link_libraries(benchmark catch_main_benchmark)
                
