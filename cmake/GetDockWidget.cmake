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
    set(QT_VERSION_MAJOR 5 CACHE STRING "Qt version major" FORCE)
    set(BUILD_STATIC TRUE CACHE BOOL "Build static library" FORCE)
    set(ADS_VERSION "4.4.0" CACHE STRING "Version" FORCE)
    set(BUILD_EXAMPLES OFF CACHE BOOL "Build examples" FORCE)
    FetchContent_MakeAvailable(dockwidget)
    add_library(dockwidget::dockwidget ALIAS "qtadvanceddocking-qt${QT_VERSION_MAJOR}")
endif()

# target: dockwidget::dockwidget