project(ProjectName)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Quick CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)

find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Quick REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Quick REQUIRED)

include_directories(${Qt5Widgets_INCLUDE_DIRS})

find_package(fmt REQUIRED)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -fopenmp -Wall -fstack-protector-strong -fext-numeric-literals -Werror=shadow -Werror=format -Werror=missing-field-initializers -Werror=return-type -Werror=array-bounds -Werror=sizeof-pointer-div -Werror=sign-compare")

add_executable(stereo-rectify
    stereo-rectify.cpp
    qml.qrc
    calib.cpp
    cam.cpp
    extr.cpp
    misc.cpp
    point3d.cpp
    stereomanager.cpp
    )


add_executable(test
    tests.cpp
    calib.cpp
    cam.cpp
    extr.cpp
    misc.cpp
    point3d.cpp
    stereomanager.cpp
    )

add_executable(stereo-sort
    stereo-sort.cpp
    misc.cpp
)

foreach(target
        stereo-rectify
        test
        stereo-sort
        )
    target_link_libraries(${target}
        ceres
        Qt5::Widgets
        ${OpenCV_LIBRARIES}
        ${RUNNINGSTATS_LIBRARIES}
        ParallelTime
        fmt::fmt
        )

    target_compile_definitions(stereo-rectify
      PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)

    target_link_libraries(stereo-rectify
      PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick)

endforeach()

find_package(libjpeg-turbo REQUIRED)
link_directories(/usr/lib/x86_64-linux-gnu/)
target_link_libraries(stereo-sort
    libraw.so
)

target_link_libraries(test
    GTest::gtest
    )
