set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)
set(Eigen_DIR /usr/lib/cmake/eigen3/)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

# set the project name and version
project(MWMP VERSION 1)

find_package (OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)

cmake_minimum_required(VERSION 3.9)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(GCC_COMPILE_FLAGS "-march=native -fopenmp")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_COMPILE_FLAGS}")

# MWMP library
file(GLOB MWMP_SRC
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB FILE_MANAGER
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/FileManager/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/FileManager/*.cpp"
)

add_library(MWMP SHARED
            ${MWMP_SRC}
            ${FILE_MANAGER})

MESSAGE(STATUS "Library MWMP, included files:
        --MWMP_SRC: ${MWMP_SRC}
        --FILE_MANAGER: ${FILE_MANAGER}")

        target_include_directories(MWMP PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/src"
                           "${PROJECT_SOURCE_DIR}/utils"
                           "${PROJECT_SOURCE_DIR}/utils/FileManager"
                           )

        target_link_libraries(MWMP PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)

# Unit tests executable
find_package (GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS} )

file(GLOB UNIT_TEST
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/*.cpp"
)

add_executable(runUnitTests
               ${UNIT_TEST}
               ${MWMP_SRC}
               ${FILE_MANAGER})

MESSAGE(STATUS "Executable runUnitTests, included files:
        --MWMP_SRC: ${MWMP_SRC}
        --FILE_MANAGER: ${FILE_MANAGER}
        --UNIT_TEST: ${UNIT_TEST}")

target_include_directories(runUnitTests PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/test/unit"
                           "${PROJECT_SOURCE_DIR}/src"
                           "${PROJECT_SOURCE_DIR}/utils"
                           "${PROJECT_SOURCE_DIR}/utils/FileManager"
                           )

target_link_libraries(runUnitTests PUBLIC ${OpenCV_LIBS} Eigen3::Eigen ${GTEST_LIBRARIES} pthread)
