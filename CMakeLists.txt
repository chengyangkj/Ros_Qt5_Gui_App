##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(ros_qt5_gui_app)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#使用c++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(PROJECT_SRC_DIR ${PROJECT_SOURCE_DIR})
##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS roscpp 
    tf
    sensor_msgs
    cv_bridge
    std_msgs
    image_transport
    )

find_package(Eigen3 REQUIRED)

#包含的头文件目录
include_directories(${catkin_INCLUDE_DIRS} ${PROJECT_SRC_DIR}/include  ${EIGEN3_INCLUDE_DIR})

catkin_package()

##############################################################################
# Qt Environment
##############################################################################

#######
## 安装多个 Qt 版本时, 可能会导致无法使用特定的 Qt 版本. 需要设置 CMAKE_PREFIX_PATH 为所用 Qt 的安装路径, 但是还需要设置 Qt[Module]_DIR(例如: QtCore_DIR) 等变量. 下面这一段代码自动会修改 QtModuleList 中的所有模块, 并自动设置 Qt[Module]_DIR变量; 同时 target_link_libraries.
set(CMAKE_PREFIX_PATH "/opt/Qt5.12.6/5.12.6/gcc_64")
set(QtModuleList 
    Core
    Gui
    Widgets
    Multimedia
    Network
)

foreach(QtModule ${QtModuleList})
    set(Qt5${QtModule}_DIR ${CMAKE_PREFIX_PATH}/lib/cmake/Qt5${QtModule})
    message("foreach-> Qt5${QtModule}_DIR : ${Qt5${QtModule}_DIR}")

    set(QT_LIBRARIES ${QT_LIBRARIES} Qt5::${QtModule})
endforeach()
message("${QT_LIBRARIES}")

find_package(Qt5 REQUIRED ${QtModuleList})

##############################################################################
# Qt Moc
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/*.hpp *.h basewidget/*.hpp *.h)
file(GLOB_RECURSE PROJECT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp basewidget/*.cpp)

QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

##############################################################################
# BINARY
##############################################################################

add_executable(${PROJECT_NAME} ${QRC_FILES} ${PROJECT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

##############################################################################
# INSTALL
##############################################################################


install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



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
