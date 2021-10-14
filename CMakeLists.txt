set(OpenCV_DIR /usr/lib/x86_64-linux-gnu/cmake/opencv4)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

# set the project name and version
project(SOMP VERSION 1)

find_package (OpenCV REQUIRED)
cmake_minimum_required(VERSION 3.9)


# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# SOMP library
file(GLOB SOMP_SRC
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

file(GLOB FILE_MANAGER
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/FileManager/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils/FileManager/*.cpp"
)

add_library(SOMP SHARED
            ${SOMP_SRC}
            ${FILE_MANAGER})

MESSAGE(STATUS "Library SOMP, included files:
        --SOMP_SRC: ${SOMP_SRC}
        --FILE_MANAGER: ${FILE_MANAGER}")

target_include_directories(SOMP PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/src"
                           "${PROJECT_SOURCE_DIR}/utils"
                           "${PROJECT_SOURCE_DIR}/utils/FileManager"
                           )

target_link_libraries(SOMP PUBLIC ${OpenCV_LIBS})

# Unit tests executable
find_package (GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS} )

file(GLOB UNIT_TEST
    "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/*.cpp"
)

add_executable(runUnitTests
               ${UNIT_TEST}
               ${SOMP_SRC}
               ${FILE_MANAGER})

MESSAGE(STATUS "Executable runUnitTests, included files:
        --SOMP_SRC: ${SOMP_SRC}
        --FILE_MANAGER: ${FILE_MANAGER}
        --UNIT_TEST: ${UNIT_TEST}")

target_include_directories(runUnitTests PUBLIC
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/test/unit"
                           "${PROJECT_SOURCE_DIR}/src"
                           "${PROJECT_SOURCE_DIR}/utils"
                           "${PROJECT_SOURCE_DIR}/utils/FileManager"
                           )

target_link_libraries(runUnitTests PUBLIC ${OpenCV_LIBS} ${GTEST_LIBRARIES} pthread)
