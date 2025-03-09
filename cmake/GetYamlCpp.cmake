include(FetchContent)
message("Fetching yaml-cpp...")

# 设置yaml-cpp版本和下载URL
set(YAML_CPP_VERSION "0.8.0" CACHE STRING "yaml-cpp version")
set(yaml-cpp_DOWNLOAD_URL
    "https://github.com/jbeder/yaml-cpp/archive/${YAML_CPP_VERSION}.tar.gz"
    CACHE STRING "")

FetchContent_Declare(
    yaml-cpp
    URL ${yaml-cpp_DOWNLOAD_URL}
)

FetchContent_GetProperties(yaml-cpp)
if(NOT yaml-cpp_POPULATED)
    # 禁用不需要的组件以加快构建速度
    set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
    set(YAML_CPP_FORMAT_SOURCE OFF CACHE BOOL "" FORCE)
    
    # 启用安装
    set(YAML_CPP_INSTALL ON CACHE BOOL "" FORCE)
    
    FetchContent_MakeAvailable(yaml-cpp)
endif()

# target: yaml-cpp::yaml-cpp