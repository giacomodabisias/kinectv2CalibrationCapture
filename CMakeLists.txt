cmake_minimum_required(VERSION 2.8)
Project(calibration_image_grabber CXX)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release) 
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")

find_package(Boost REQUIRED)
set(BOOST_LIBS boost_system boost_filesystem)

find_package(OpenCV     REQUIRED)
find_package(Freenect2  REQUIRED)

add_executable(calibration_image_grabber main.cpp )

target_link_libraries(calibration_image_grabber ${FREENECT2_LIBRARY} ${OpenCV_LIBS} ${BOOST_LIBS})

