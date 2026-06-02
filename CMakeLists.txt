cmake_minimum_required(VERSION 3.16)
project(DK1Tracker VERSION 0.1.0 LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# -fPIC is needed for the shared library; bundle the example as a macOS .app.
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

# Create the shared library
add_library(DK1Tracker SHARED ${LIB_SOURCES})

target_link_libraries(DK1Tracker PRIVATE ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})
target_include_directories(DK1Tracker PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Example – build as a normal command line binary
add_executable(dk1_dump examples/dk1_dump.c)
target_link_libraries(dk1_dump PRIVATE DK1Tracker)

# ---------------------------------------------------------------------------
# Install + distribution
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS DK1Tracker
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS dk1_dump
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})

# "dist" target: produce a pure source tarball (no compiled artifacts).
# The staged layout mirrors the project root, e.g.
# build/_dist/DK1Tracker-0.1.0/:
#   CMakeLists.txt, README.md, .gitignore
#   src/, examples/, docs/, include/
#
# The recipient rebuilds with the usual cmake -S . -B build dance.
set(DIST_NAME    "${PROJECT_NAME}-${PROJECT_VERSION}")
set(DIST_STAGE   "${CMAKE_BINARY_DIR}/_dist")
set(DIST_PREFIX  "${DIST_STAGE}/${DIST_NAME}")
set(DIST_ARCHIVE "${CMAKE_BINARY_DIR}/${DIST_NAME}.tar.gz")

# Per-file copies need a separate -E copy_if_different invocation per
# source; directories are handled by -E copy_directory.
set(DIST_SOURCE_FILES
    CMakeLists.txt
    README.md
    .gitignore
)
set(DIST_SOURCE_DIRS
    src
    examples
    docs
    include
)

# Build the per-file copy commands and the per-directory copy commands
# at configure time so the recipe stays declarative.
set(_dist_file_copies)
foreach(_f IN LISTS DIST_SOURCE_FILES)
    list(APPEND _dist_file_copies
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${CMAKE_SOURCE_DIR}/${_f}" "${DIST_PREFIX}/${_f}"
    )
endforeach()

set(_dist_dir_copies)
foreach(_d IN LISTS DIST_SOURCE_DIRS)
    list(APPEND _dist_dir_copies
        COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${CMAKE_SOURCE_DIR}/${_d}" "${DIST_PREFIX}/${_d}"
    )
endforeach()

add_custom_target(dist
    # 1) Wipe and re-create the staging tree.
    COMMAND ${CMAKE_COMMAND} -E rm -rf "${DIST_STAGE}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DIST_PREFIX}"
    # 2) Copy the build + source files into the staged root.
    ${_dist_file_copies}
    ${_dist_dir_copies}
    # 3) Package the staged tree, with cwd set to its parent so the
    #    tarball's top-level entry is the versioned directory.
    COMMAND ${CMAKE_COMMAND} -E chdir "${DIST_STAGE}"
            ${CMAKE_COMMAND} -E tar czf "${DIST_ARCHIVE}" "${DIST_NAME}"
    BYPRODUCTS "${DIST_ARCHIVE}"
    COMMENT "Packaging ${DIST_ARCHIVE}"
    VERBATIM
)
