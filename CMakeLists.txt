project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

######################################################CMake Required module ######################################
include(ExternalProject)  # required cmake module
##################################################################################################################

######################################################Get and build External projects ######################################
ExternalProject_Add(uWS
${uWS}
PREFIX            ${uWS}
GIT_REPOSITORY  "https://github.com/uWebSockets/uWebSockets"
GIT_TAG			  v0.13.0
INSTALL_DIR       ${CMAKE_CURRENT_BINARY_DIR}/ext/${uWS}
CMAKE_CACHE_ARGS
		-DBUILD_SHARED_LIBS:BOOL=ON
		-DENABLE_STATIC_RUNTIME:BOOL=OFF
		-DBUILD_EXAMPLES:BOOL=ON
		-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>

BUILD_COMMAND     ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release --target INSTALL
)


##################################################################################################################
######################################################### PACKAGES #############################################

find_package (Eigen3 3.3 REQUIRED )
find_package(OpenSSL REQUIRED)
find_package(unofficial-libuv CONFIG REQUIRED)
find_package(ZLIB REQUIRED)
################################################################################################################


set(sources 
    src/main.cpp 
    src/tools.cpp 
    src/FusionEKF.cpp 
    src/kalman_filter.cpp 
    )


#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

#include_directories(/usr/local/include)
#include_directories(/usr/local/opt/openssl/include)
#link_directories(/usr/local/lib)
#link_directories(/usr/local/opt/openssl/lib)
#link_directories(/usr/local/Cellar/libuv/1*/lib)

#endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(ExtendedKF ${sources})

target_include_directories(ExtendedKF PRIVATE 
							${CMAKE_CURRENT_SOURCE_DIR}/include
							${CMAKE_CURRENT_BINARY_DIR}/uWS-prefix/src/uWS/src
							${OPENSSL_INCLUDE_DIR}
							)

target_link_libraries(ExtendedKF 
					  #z
					  ZLIB::ZLIB
					  #ssl
					  OpenSSL::SSL OpenSSL::Crypto 
					  #uv
					  unofficial::libuv::libuv 
					  #uWS
					 ${CMAKE_CURRENT_BINARY_DIR}/ExtendedKF.lib
					  Eigen3::Eigen
					  )

