include(FetchContent)
message("Fetching dockwidget...")

set(dockwidget_DOWNLOAD_URL
    "https://github.com/githubuser0xFFFF/Qt-Advanced-Docking-System/archive/refs/tags/4.4.0.zip"
    CACHE STRING "")

FetchContent_Declare(
    dockwidget
    URL ${dockwidget_DOWNLOAD_URL}
)

FetchContent_GetProperties(dockwidget)
if(NOT dockwidget_POPULATED)
    set(QT_VERSION_MAJOR 5)
    set(BUILD_STATIC TRUE)
    set(ADS_VERSION "4.4.0")
    set(BUILD_EXAMPLES OFF)
    FetchContent_MakeAvailable(dockwidget)
    add_library(dockwidget::dockwidget ALIAS "qtadvanceddocking-qt${QT_VERSION_MAJOR}")
endif()

# target: dockwidget::dockwidget