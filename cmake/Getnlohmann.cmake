include(FetchContent)
    
message("Fetching nlohmann_json...")

# 设置nlohmann_json版本和下载URL
set(nlohmann_json_DOWNLOAD_URL
    "https://github.com/nlohmann/json/archive/refs/tags/v3.12.0.tar.gz"
    CACHE STRING "")

FetchContent_Declare(
    nlohmann_json
    URL ${nlohmann_json_DOWNLOAD_URL}
)

FetchContent_GetProperties(nlohmann_json)
if(NOT nlohmann_json_POPULATED)
    
    FetchContent_MakeAvailable(nlohmann_json)
endif()

# target: nlohmann_json::nlohmann_json