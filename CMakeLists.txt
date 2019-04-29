project(mammoth C CXX)
cmake_minimum_required(VERSION 3.5)

set(QT5_ROOT_PATH D:/Qt/Qt5.8.0)
set(PCL_ROOT_PATH D:/PCL1.8.0)
set(OpenCV_ROOT_PATH D:/opencv/opencv)
set(PCAP_ROOT_PATH D:/WinPacp/WpdPack)
set(GLEW_ROOT_PATH D:/glew)

set(COMMON_PATH "common")
set(IO_PATH "io")
set(PREPROCESS_PATH "preprocess")
set(UTIL_PATH "util")
set(ALGORITHM_PATH "algorithm")
set(VISIUALIZATION_PATH "visualization")
#set(SLAM_PATH "slam")
set(APP_PATH "app")
set(TEST_PATH "test")

file(GLOB_RECURSE MY_HEADERS
    "${COMMON_PATH}/include/*.h" "${COMMON_PATH}/include/*.hpp"
    "${IO_PATH}/include/*.h" "${IO_PATH}/include/*.hpp"
    "${PREPROCESS_PATH}/include/*.h" "${PREPROCESS_PATH}/include/*.hpp"
    "${UTIL_PATH}/include/*.h" "${UTIL_PATH}/include/*.hpp"
    "${ALGORITHM_PATH}/include/*.h" "${ALGORITHM_PATH}/include/*.hpp"
	#"${SLAM_PATH}/include/*.h" "${SLAM_PATH}/include/*.hpp"
    "${VISIUALIZATION_PATH}/include/*.h" "${VISIUALIZATION_PATH}/include/*.hpp"
    "${APP_PATH}/include/*.h" "${APP_PATH}/include/*.hpp"
    "${TEST_PATH}/include/*.h" "${TEST_PATH}/include/*.hpp"
)

file(GLOB_RECURSE MY_SRCS
    "${COMMON_PATH}/src/*.c" "${COMMON_PATH}/src/*.cpp"
    "${IO_PATH}/src/*.c" "${IO_PATH}/src/*.cpp"
    "${PREPROCESS_PATH}/src/*.c" "${PREPROCESS_PATH}/src/*.cpp"
    "${UTIL_PATH}/src/*.c" "${UTIL_PATH}/src/*.cpp"
    "${ALGORITHM_PATH}/src/*.c" "${ALGORITHM_PATH}/src/*.cpp"
	#"${SLAM_PATH}/src/*.c" "${SLAM_PATH}/src/*.cpp"
    "${VISIUALIZATION_PATH}/src/*.c" "${VISIUALIZATION_PATH}/src/*.cpp"
    "${APP_PATH}/src/*.c" "${APP_PATH}/src/*.cpp"
    #"${TEST_PATH}/src/*.c" "${TEST_PATH}/src/*.cpp"
)

message("MY_HEADERS : ${MY_HEADERS}")
message("MY_SRCS : ${MY_SRCS}")

file(GLOB_RECURSE MY_LIBS
    "${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${IO_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${PREPROCESS_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${UTIL_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${ALGORITHM_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${VISIUALIZATION_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${APP_PATH}/lib/*.lib"
    "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_PATH}/lib/*.lib"
)

if(UNIX)
    include(cmake/configuration.cmake)
endif()

if(WIN32)
    set(QT5_DIR ${QT5_ROOT_PATH}/5.8/mingw53_32)
    set(GLEW_DIR ${GLEW_ROOT_PATH})
    set(PCL_DIR ${PCL_ROOT_PATH}/cmake)
    set(OpenCV_DIR ${OpenCV_ROOT_PATH}/build)
    set(PCAP_DIR ${PCAP_ROOT_PATH})
    set(PCAP_INCLUDE_DIR ${PCAP_DIR}/Include)
    set(CMAKE_PREFIX_PATH ${QT5_DIR};${GLEW_DIR})

    file(GLOB_RECURSE PCAP_HEADERS "${PCAP_DIR}/Include/*.h" "${PCAP_DIR}/Include/pcap/*.h")
    file(GLOB_RECURSE PCAP_LIBS "${PCAP_DIR}/Lib/x64/*.lib")
    file(GLOB_RECURSE GLEW_LIBS "${GLEW_ROOT_PATH}/lib/*.lib")
elseif(UNIX)
    set(PCL_DIR /usr/local/pcl/share/pcl-1.8)
    find_package(Boost COMPONENTS filesystem REQUIRED)
endif()

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(MRPT REQUIRED base;hwdrivers;gui;maps;bayes;slam)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${PCAP_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
#include_directories(${KALMAN_FILTER_INCLUDE_DIR})
# include_directories(${MRPT_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${COMMON_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${IO_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${PREPROCESS_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${UTIL_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${ALGORITHM_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${VISIUALIZATION_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${APP_PATH}/include/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${TEST_PATH}/include/)


if(WIN32)
    add_executable(${PROJECT_NAME} ${MY_SRCS} ${MY_HEADERS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${PCAP_LIBS} ${GLEW_LIBS} ${MY_LIBS})
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O3")
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
elseif(UNIX)
    add_definitions(-Wno-return-local-addr -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-comment -Wno-return-type -Wno-format)
    add_executable(${PROJECT_NAME} ${MY_SRCS} ${MY_HEADERS})
    target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} pthread pcap)
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} pcap ${CMAKE_CURRENT_SOURCE_DIR}/lib/libptvisualizier.so ${MRPT_LIBS})
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
endif()
