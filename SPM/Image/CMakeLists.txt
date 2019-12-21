cmake_minimum_required(VERSION 3.5)
project(Image)
#add_definitions(-DNO_XML -DCMAKE_BUILD_TYPE=RELEASE)
add_definitions(-DNO_XML)

#Set the correct output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/bin")

## Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -std=c++11")
set(CMAKE_EXE_LINKER_FLAGS "-s")  ## Strip binary

set(SOURCE_FILES
        bilateral_filtering/fft_3D/convolution_3D.h
        bilateral_filtering/fft_3D/fill_3D.h
        bilateral_filtering/fft_3D/support_3D.cpp
        bilateral_filtering/fft_3D/support_3D.h
        bilateral_filtering/include/array.h
        bilateral_filtering/include/array_n.h
        bilateral_filtering/include/chrono.h
        bilateral_filtering/include/fast_color_bf.h
        bilateral_filtering/include/fft_3D.h
        bilateral_filtering/include/geom.h
        bilateral_filtering/include/linear_bf.h
        bilateral_filtering/include/math_tools.h
        bilateral_filtering/include/mixed_vector.h
        bilateral_filtering/include/msg_stream.h
        ../Statistics/Ransac.h
        ../Statistics/RNG.h
        ../Math/Algebra.h
        ../Geometry/Edge.h
        ../Geometry/Face.h
        ../Geometry/GeometricObject.cpp
        ../Geometry/GeometricObject.h
        ../Geometry/GeometryExporter.cpp
        ../Geometry/GeometryExporter.h
        ../Geometry/GeometryProcessing.cpp
        ../Geometry/GeometryProcessing.h
        ../General/FileUtilities.h
        ../General/Utilities.h
        Color.cpp
        Color.h
        Image.cpp
        Image.h
        ImageProcessing.cpp
        ImageProcessing.h
        main.cpp)

include_directories(.)
include_directories(bilateral_filtering)
include_directories(bilateral_filtering/fft_3D)
include_directories(bilateral_filtering/include)

include_directories(../Statistics)
include_directories(../Math)
include_directories(../Geometry)
include_directories(../General)


include_directories(/usr/include/eigen3)

# Configuring packages
## ImageMagic
find_package(ImageMagick)
find_package(ImageMagick COMPONENTS convert)
find_package(ImageMagick COMPONENTS convert mogrify display)
find_package(ImageMagick COMPONENTS Magick++)
find_package(ImageMagick COMPONENTS Magick++ convert)
#MESSAGE(STATUS "ImageMagick: " ${ImageMagick_INCLUDE_DIRS})
include_directories(${ImageMagick_INCLUDE_DIRS})

## libfftw3
set (CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")	#include a script to find LibFFTW3
set(LIBFFTW3_PRECISION DOUBLE)				#using double precision since some functions are not implemented in float precision
find_package(LibFFTW3 REQUIRED)
include_directories(${LIBFFTW3_INCLUDE_DIR})
#include_directories(/usr/include/fftw3)

## OpenCV
find_package(OpenCV REQUIRED)
#MESSAGE(STATU "OpenCV LIBS:" ${OpenCV_LIBS})
#MESSAGE(STATU "OpenCV Includes:" ${OpenCV_INCLUDE_DIRS})
# Moving all #include "opencv.hpp" to #include "opencv2/opencv.hpp"
#include_directories(/usr/include/)

#MPFR
find_package(MPFR REQUIRED)
include_directories(${MPFR_INCLUDES})

#GMP
find_package(GMP REQUIRED)
include_directories(${GMP_INCLUDES})

#CGAL
find_package(CGAL REQUIRED)
include_directories(${CGAL_INCLUDE_DIR})
#MESSAGE(STATUS "CGAL: " ${CGAL_INCLUDE_DIR})


add_executable(Image ${SOURCE_FILES})
target_link_libraries(  Image
                        ${ImageMagick_LIBRARIES}
                        ${LIBFFTW3_LIBRARY}
                        ${OpenCV_LIBS}
                        ${MPFR_LIBRARIES}
                        ${GMP_LIBRARIES}
                        ${CGAL_LIBRARIES})
