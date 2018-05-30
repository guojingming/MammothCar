project(mammoth C CXX)
cmake_minimum_required(VERSION 3.5)

include(cmake/configuration.cmake)


if(WIN32)
    set(PCL_DIR D:/PCL 1.8.0/cmake)
    set(BOOST_ROOT D:/PCL 1.8.0/3rdParty/Boost)
elseif(UNIX)
    set(PCL_DIR /usr/local/pcl/share/pcl-1.8)
    #set(KALMAN_FILTER_INCLUDE_DIR /usr/local/include/mrpt/bayes/include)
endif()

find_package(PCL REQUIRED)
find_package(Boost COMPONENTS filesystem REQUIRED)
message("aaaaaaaaaaaaaaaaaaaaaaaa:${PCL_INCLUDE_DIRS}")
message("bbbbbbbbbbbbbbbbbbbbbbbb:${PCL_LIBRARY_DIRS}")

# find_package(PCL REQUIRED)
# find_package(Boost COMPONENTS filesystem REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(MRPT REQUIRED base;hwdrivers;gui;maps;bayes;slam)

# include_directories(${OpenCV_INCLUDE_DIRS})
# include_directories(${Boost_INCLUDE_DIRS})
# include_directories(${PCL_INCLUDE_DIRS})
# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# #include_directories(${KALMAN_FILTER_INCLUDE_DIR})

# include_directories(${MRPT_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})

# file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
# file(GLOB_RECURSE SRCS "src/*.c" "src/*.cpp")


if(WIN32)
    add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS})
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} pcap)
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
elseif(UNIX)
    add_definitions(-Wno-return-local-addr -Wno-unknown-pragmas -Wno-sign-compare -Wno-unused-variable -Wno-unused-but-set-variable -Wno-comment -Wno-return-type -Wno-format)
    add_executable(${PROJECT_NAME} ${SRCS} ${HEADERS})
    #target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} pthread pcap)
    target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} pcap ${CMAKE_CURRENT_SOURCE_DIR}/lib/libptvisualizier.so ${MRPT_LIBS})
    if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
    endif(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
endif()