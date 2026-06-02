cmake_minimum_required(VERSION 3.16)
project(DK1Tracker VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Framework/Shared Library settings
set(CMAKE_MACOSX_BUNDLE TRUE)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

# Find macOS Frameworks
find_library(IOKIT_FRAMEWORK IOKit REQUIRED)
find_library(COREFOUNDATION_FRAMEWORK CoreFoundation REQUIRED)

# Include directories
include_directories(include)

# Library sources
set(LIB_SOURCES
    src/DK1Tracker.c
    src/DK1Parser.c
    src/DK1Estimator.c
    src/DK1RingBuffer.c
    src/DK1HIDBackend_mac.c
)

# Create the library
add_library(DK1Tracker SHARED ${LIB_SOURCES})

target_link_libraries(DK1Tracker PRIVATE ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})
target_include_directories(DK1Tracker PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

# Example
add_executable(dk1_dump examples/dk1_dump.c)
target_link_libraries(dk1_dump PRIVATE DK1Tracker)
