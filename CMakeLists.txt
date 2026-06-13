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
    src/DK1HeadModel.c
    src/DK1Prediction.c
    src/DK1RingBuffer.c
    src/DK1Config.c
    src/DK1Distortion.c
    src/DK1MetalDistortion.c
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
# Example – build as a normal command line binary
add_executable(dk1_state_60hz examples/dk1_state_60hz.c)
target_link_libraries(dk1_state_60hz PRIVATE DK1Tracker)
# Tool – collect stationary gyro bias and save it to the DK1 config.
add_executable(dk1_calibrate examples/calibrate.c)
target_link_libraries(dk1_calibrate PRIVATE DK1Tracker)
target_include_directories(dk1_calibrate PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

include(CTest)
if(BUILD_TESTING)
    add_executable(dk1_head_model_test tests/test_head_model.c)
    target_link_libraries(dk1_head_model_test PRIVATE DK1Tracker)
    target_include_directories(dk1_head_model_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_head_model COMMAND dk1_head_model_test)

    add_executable(dk1_estimator_test tests/test_estimator.c)
    target_link_libraries(dk1_estimator_test PRIVATE DK1Tracker)
    target_include_directories(dk1_estimator_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_estimator COMMAND dk1_estimator_test)

    add_executable(dk1_config_test tests/test_config.c)
    target_link_libraries(dk1_config_test PRIVATE DK1Tracker)
    target_include_directories(dk1_config_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_config COMMAND dk1_config_test)

    add_executable(dk1_parser_test tests/test_parser.c)
    target_link_libraries(dk1_parser_test PRIVATE DK1Tracker)
    target_include_directories(dk1_parser_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_parser COMMAND dk1_parser_test)

    add_executable(dk1_prediction_test tests/test_prediction.c)
    target_link_libraries(dk1_prediction_test PRIVATE DK1Tracker)
    target_include_directories(dk1_prediction_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_prediction COMMAND dk1_prediction_test)

    add_executable(dk1_distortion_mesh_test tests/test_distortion_mesh.c)
    target_link_libraries(dk1_distortion_mesh_test PRIVATE DK1Tracker)
    target_include_directories(dk1_distortion_mesh_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_distortion_mesh COMMAND dk1_distortion_mesh_test)

    add_executable(dk1_eye_projection_test tests/test_eye_projection.c)
    target_link_libraries(dk1_eye_projection_test PRIVATE DK1Tracker)
    target_include_directories(dk1_eye_projection_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_eye_projection COMMAND dk1_eye_projection_test)

    add_executable(dk1_metal_distortion_test tests/test_metal_distortion.c)
    target_link_libraries(dk1_metal_distortion_test PRIVATE DK1Tracker)
    target_include_directories(dk1_metal_distortion_test PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    add_test(NAME dk1_metal_distortion COMMAND dk1_metal_distortion_test)
endif()

option(DK1_BUILD_METAL_LIBRARY "Build the DK1 Metal distortion shader library" ON)
set(DK1_METAL_SHADER_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/shaders/DK1Distortion.metal")
set(DK1_HAVE_METAL_TOOLCHAIN OFF)

find_program(XCRUN_EXECUTABLE xcrun)
if(DK1_BUILD_METAL_LIBRARY AND XCRUN_EXECUTABLE)
    execute_process(
        COMMAND "${XCRUN_EXECUTABLE}" -sdk macosx metal -v
        RESULT_VARIABLE DK1_METAL_CHECK_RESULT
        OUTPUT_QUIET
        ERROR_QUIET
    )
    if(DK1_METAL_CHECK_RESULT EQUAL 0)
        set(DK1_HAVE_METAL_TOOLCHAIN ON)
    else()
        message(STATUS "Metal compiler not usable; skipping DK1Distortion.metallib build")
    endif()
elseif(DK1_BUILD_METAL_LIBRARY)
    message(STATUS "xcrun not found; skipping DK1Distortion.metallib build")
endif()

if(DK1_HAVE_METAL_TOOLCHAIN)
    set(DK1_METAL_AIR "${CMAKE_CURRENT_BINARY_DIR}/DK1Distortion.air")
    set(DK1_METAL_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/DK1Distortion.metallib")

    add_custom_command(
        OUTPUT "${DK1_METAL_AIR}"
        COMMAND "${XCRUN_EXECUTABLE}" -sdk macosx metal
                -c "${DK1_METAL_SHADER_SOURCE}"
                -o "${DK1_METAL_AIR}"
        DEPENDS "${DK1_METAL_SHADER_SOURCE}"
        VERBATIM
    )
    add_custom_command(
        OUTPUT "${DK1_METAL_LIBRARY}"
        COMMAND "${XCRUN_EXECUTABLE}" -sdk macosx metallib
                "${DK1_METAL_AIR}"
                -o "${DK1_METAL_LIBRARY}"
        DEPENDS "${DK1_METAL_AIR}"
        VERBATIM
    )
    add_custom_target(DK1DistortionMetallib ALL DEPENDS "${DK1_METAL_LIBRARY}")
endif()

# ---------------------------------------------------------------------------
# Install + distribution
# ---------------------------------------------------------------------------
include(GNUInstallDirs)

install(TARGETS DK1Tracker
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(TARGETS dk1_dump dk1_calibrate
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES README.md DESTINATION ${CMAKE_INSTALL_DOCDIR})
install(FILES shaders/DK1Distortion.metal
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/shaders
)
if(DK1_HAVE_METAL_TOOLCHAIN)
    install(FILES "${DK1_METAL_LIBRARY}"
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
    )
endif()

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
    shaders
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
