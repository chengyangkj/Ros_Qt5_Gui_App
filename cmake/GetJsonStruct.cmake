include(FetchContent)
message("Fetching json_struct...")

set(json_struct_DOWNLOAD_URL
    "https://github.com/jorgen/json_struct/archive/refs/tags/1.0.1.zip"
    CACHE STRING "")

FetchContent_Declare(
    json_struct
    URL ${json_struct_DOWNLOAD_URL}
)

FetchContent_GetProperties(json_struct)
if(NOT json_struct_POPULATED)
    FetchContent_MakeAvailable(json_struct)
endif()

# target: json_struct::json_struct