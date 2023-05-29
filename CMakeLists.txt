project(ProjectName)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(Ceres REQUIRED)

find_package(RunningStats REQUIRED)

find_package(Boost COMPONENTS system filesystem iostreams REQUIRED)

find_package(GTest REQUIRED)


set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -Werror=return-type")

add_executable(stereo-rectify
    stereo-rectify.cpp
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

add_executable(test
    tests.cpp
    calib.cpp
    cam.cpp
    extr.cpp
    misc.cpp
    point3d.cpp
    )

target_link_libraries(test
    ceres
    GTest::gtest
    ${OpenCV_LIBRARIES}
    ${RUNNINGSTATS_LIBRARIES}
    )
