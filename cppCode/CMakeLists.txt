cmake_minimum_required(VERSION 3.12)
project(cppremoteapi)

INCLUDE(FindPkgConfig)
find_package( OpenCV REQUIRED )
# include_directories( ${OpenCV_INCLUDE_DIRS} )

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

IF(CMAKE_COMPILER_IS_GNUCC)
SET(CMAKE_CXX_FLAGS
"${CMAKE_CXX_FLAGS} -Wall -std=c++0x -DBUILD_SHARED_LIBS=OFF")
ENDIF(CMAKE_COMPILER_IS_GNUCC)

# definicoes da remoteAPI para CPP
add_definitions(-DNON_MATLAB_PARSING)
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)

set(SOURCES
    remoteApi/extApi.c
    #remoteApi/extApiCustom.c
    remoteApi/extApiPlatform.c
    source/control.cpp
    source/colorSearch.cpp
    source/actuator.cpp
    source/vision.cpp
)
 
set(HEADERS
    remoteApi/extApi.h
    #remoteApi/extApiCustom.h
    remoteApi/extApiInternal.h
    remoteApi/extApiPlatform.h
    include/v_repConst.h
    include/v_repLib.h
    include/control.hpp
    include/colorSearch.hpp
    include/actuator.hpp
    include/vision.hpp
)
include_directories("include/")
include_directories("remoteApi/")

find_package (Threads)

add_executable(cppremoteapi 
               main.cpp 
               ${SOURCES}
	       ${HEADERS})
	       
target_link_libraries (cppremoteapi ${CMAKE_THREAD_LIBS_INIT} ${OpenCV_LIBS})

install(TARGETS cppremoteapi RUNTIME DESTINATION bin)
