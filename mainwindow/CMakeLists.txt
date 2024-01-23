include(TargetNameHelper)
GetTargetName()

find_package(Qt5 COMPONENTS Widgets REQUIRED)

file(GLOB_RECURSE SOURCE_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR} src/*.cpp src/*.cc)
file(GLOB_RECURSE UI_FILE CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}  ui/*.ui)
file(GLOB_RECURSE QRC_FILE  CONFIGURE_DEPENDS ${CMAKE_SOURCE_DIR}  resource/*.qrc)
file(GLOB_RECURSE INCLUDE_FILE CONFIGURE_DEPENDS  ${CMAKE_SOURCE_DIR}  include/*.h *.hpp)

####qt moc
qt5_wrap_ui(QT_UI_HPP ${UI_FILE})
qt5_wrap_cpp(QT_MOC_HPP ${INCLUDE_FILE})
qt5_add_resources(QT_RESOUCE_HPP ${QRC_FILE})
#向可执行程序添加源文件
add_library(${TARGET_NAME} STATIC
    ${SOURCE_FILE}
    ${INCLUDE_FILE}
    ${UI_FILE}
    ${QT_MOC_HPP}
    ${QRC_FILE}
    ${QT_RESOUCE_HPP}
  )
  #项目包含的头文件路径
target_include_directories(
  ${TARGET_NAME}
  PUBLIC
	include/
  include/display/
 ${CMAKE_CURRENT_BINARY_DIR}
)
target_link_libraries(${TARGET_NAME} Qt5::Widgets Qt5::Core qt${QT_VERSION_MAJOR}advanceddocking json_struct common basic channel)

