##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(ros_qt5_gui_app)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#添加c++11的支持
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
##############################################################################
# Catkin
##############################################################################
# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS rviz roscpp sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    )

include_directories(${catkin_INCLUDE_DIRS} ./include)
# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package()

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
#rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here

#######
## 安装多个 Qt 版本时, 可能会导致无法使用特定的 Qt 版本. 需要设置 CMAKE_PREFIX_PATH 为所用 Qt 的安装路径, 但是还需要设置 Qt[Module]_DIR(例如: QtCore_DIR) 等变量. 下面这一段代码自动会修改 ModuleList 中的所有模块, 并自动设置 Qt[Module]_DIR变量; 同时 target_link_libraries.
set(CMAKE_PREFIX_PATH "/opt/Qt5.12.6/5.12.6/gcc_64")
set(ModuleList 
    Core
    Gui
    Widgets
    Multimedia
    Network
)

foreach(QtModule ${ModuleList})
    set(Qt5${QtModule}_DIR ${CMAKE_PREFIX_PATH}/lib/cmake/Qt5${QtModule})
    message("foreach-> Qt5${QtModule}_DIR : ${Qt5${QtModule}_DIR}")

    set(QT_LIBRARIES ${QT_LIBRARIES} Qt5::${QtModule})
endforeach()
message("${QT_LIBRARIES}")

find_package(Qt5 REQUIRED ${ModuleList})
#######

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
#file(GLOB_RECURSE QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h basewidget/*.hpp *.h)

#set(QRC_SOURCE_FILES image.qrc)
#qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})
#add_library(${PROJECT_NAME}
#${QRC_FILES})

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp basewidget/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(ros_qt5_gui_app ${QRC_FILES} ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(ros_qt5_gui_app ${QT_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS ros_qt5_gui_app RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#############################################################################
#HTML 将html文件拷贝到可执行文件同级目录
#############################################################################
#file(REMOVE_RECURSE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/html)
#file(REMOVE_RECURSE ${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/html)
#file(COPY ${PROJECT_SOURCE_DIR}/html  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
#install(DIRECTORY ${PROJECT_SOURCE_DIR}/html DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
#message("PROJECT_SOURCE_DIR:${CMAKE_INSTALL_PREFIX}")

#############################################################################
#DEBUG && RELEASE
#############################################################################
#如果要生成debug版本
#SET(CMAKE_BUILD_TYPE "Debug")
#如果要生成release版本
#SET(CMAKE_BUILD_TYPE "Release")
#添加的支持debug和release支持
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
#SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
