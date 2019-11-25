cmake_minimum_required(VERSION 3.0.0)
project(hash-real-data VERSION 0.1.0)

set(CMAKE_REQUIRED_FLAGS -std=c++17)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
include(CTest)
enable_testing()
add_subdirectory(tests)
add_library(PrimitivePlanner
  PrimitiveEnvironment.cpp
  PrimitivesCollection.cpp
  PrimitiveAdapter.cpp
  State.cpp
  StateDis.cpp
  )

#include_directories($(hash-real-data_SOURCE_DIR)/include)
find_package(ompl REQUIRED)
find_package (Eigen3 REQUIRED)
find_package(Boost 1.36.0 REQUIRED COMPONENTS serialization filesystem)
if(Boost_FOUND)
  include_directories(${Boost_INCLUDE_DIRS})
endif()
include_directories(${OMPL_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(hash-real-data hash-real-data.cpp)
add_executable(statedis StateDis.cpp)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
target_link_libraries(hash-real-data Boost::serialization)

