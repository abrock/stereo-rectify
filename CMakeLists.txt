project(ProjectName)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)

find_package(RunningStats REQUIRED)

find_package(Boost COMPONENTS system filesystem iostreams REQUIRED)

find_package(GTest REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick REQUIRED)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -Werror=return-type -fopenmp ")

add_executable(stereo-rectify
    stereo-rectify.cpp
    qml.qrc
    calib.cpp
    cam.cpp
    extr.cpp
    misc.cpp
    point3d.cpp
    )

target_link_libraries(stereo-rectify
    ceres
    ${OpenCV_LIBRARIES}
    ${RUNNINGSTATS_LIBRARIES}
    )

target_compile_definitions(stereo-rectify
  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(stereo-rectify
  PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick)

add_executable(test
    tests.cpp
    calib.cpp
    cam.cpp
    extr.cpp
    misc.cpp
    point3d.cpp
    )

target_compile_definitions(test
  PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

target_link_libraries(test
    ceres
    GTest::gtest
    ${OpenCV_LIBRARIES}
    ${RUNNINGSTATS_LIBRARIES}
    )

target_link_libraries(test
  PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick)
