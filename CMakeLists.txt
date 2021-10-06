cmake_minimum_required(VERSION 3.9)

# set the project name and version
project(SOMP VERSION 0.1)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

include_directories(src)

file(GLOB SOMP_SRC
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp"
     "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(SOMP SHARED
            ${SOMP_SRC})

MESSAGE(STATUS "Files included:
        --SOMP_SRC: ${SOMP_SRC}")

# add the executable
target_include_directories(SOMP PUBLIC 
                           "${PROJECT_BINARY_DIR}"
                           "${PROJECT_SOURCE_DIR}/src"
                           "${PROJECT_SOURCE_DIR}/test"
                           )
#target_link_libraries(SOMP PUBLIC MatrixOperations)
