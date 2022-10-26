cmake_minimum_required(VERSION 3.0.2)
project(mpbb_tests)

#add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(MPBB_PROJ_DIRECTORY ${PROJECT_SOURCE_DIR} )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(OpenMP REQUIRED)
find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


file(GLOB_RECURSE SRC_FILES 
 src/*.cpp 
)
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MPBB_PROJ_DIRECTORY}/include
)


# build mpbb lib 
add_library(mpbb SHARED ${SRC_FILES})
target_link_libraries(mpbb 

-dpthread
${Boost_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
)

set_target_properties( mpbb PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${MPBB_PROJ_DIRECTORY}/lib 
)

add_executable(mpbb_run ${MPBB_PROJ_DIRECTORY}/example/mpbb_run.cpp ${SRC_FILES} ) 
target_link_libraries( mpbb_run mpbb) 
set_target_properties( mpbb_run PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#add_executable(ffp_vs_wfd ${FD_PROJ_DIRECTORY}/example/ffp_vs_wfd.cpp ${SRC_FILES} ) 
#target_link_libraries(ffp_vs_wfd ${OpenCV_LIBS}) 
#set_target_properties( ffp_vs_wfd PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
