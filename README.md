# DK1Tracker

A native macOS C framework for interfacing with the Oculus Rift DK1 tracker.

## Purpose
This library provides a user-space HID interface to the DK1 tracker, allowing for the retrieval of accelerometer, gyroscope, and magnetometer data.

## Technical Details
- **Backend**: Uses Apple's native IOKit/CoreFoundation HID APIs (`IOHIDManager`).
- **Language**: C11.
- **Dependencies**: `IOKit.framework`, `CoreFoundation.framework`.
- **No Homebrew**: Built as a native macOS framework without external dependency managers.
- **Architecture**: User-space HID framework (not a DriverKit or kernel driver).

## Build Instructions
Use CMake to build the project:

```bash
cmake -S . -B build
cmake --build build
```

## Configuration
At tracker creation, the library reads:

```text
~/.OculusRiftDK1/config.txt
```

The file contains five whitespace-separated integers:

```text
left_dial right_dial grid_width grid_height ipd_mm
```

Example:

```text
5 5 64 64 64
```

If the file is missing, the library uses `5 5 64 64 64`. Legacy four-value
files are still accepted and default `ipd_mm` to `64`. Dial values must be in
the range `0..10`; grid dimensions must be positive; `ipd_mm` must be in the
range `40..90`.

The dial and IPD values are used at tracker creation to precompute one
distortion mesh per eye. Each mesh contains full-framebuffer screen positions
plus separate red, green, and blue tangent-eye-angle sample coordinates. Mesh
pointers returned by `dk1_tracker_get_distortion_mesh` are owned by the tracker
and stay valid until `dk1_tracker_destroy`.

## Metal Distortion Shader
The build produces `build/DK1Distortion.metallib` when the Xcode Metal command
line toolchain is available. If CMake reports that the Metal compiler is not
usable, install it with:

```bash
xcodebuild -downloadComponent MetalToolchain
```

The shader entry points are:

```text
dk1_distortion_vertex
dk1_distortion_fragment
```

Use `dk1_metal_distortion_copy_vertices` to pack a `DK1DistortionMesh` into the
float vertex layout consumed by the shader. Use
`dk1_metal_distortion_make_eye_texture_uniforms` to create the tangent-eye-angle
to source-texture UV transform for a single eye texture. Draw each eye mesh as
an indexed triangle list, binding the packed vertex buffer at Metal buffer slot
0, `DK1MetalDistortionUniforms` at slot 1, and the rendered eye texture at
texture slot 0.

## Distribution
A `dist` target is provided that stages a clean install of the library, headers, README, and `dk1_dump` example into a versioned directory, then packages it as a gzip-compressed tarball:

```bash
cmake --build build --target dist
# -> build/DK1Tracker-<version>.tar.gz
```

The tarball layout follows the standard GNUInstallDirs tree (`lib/`, `bin/`, `include/`, `share/doc/<project>/`), so it can be extracted and installed system-wide with `cmake --install`.

## Important Notes
* The parser bit layout for packed motion samples needs verification against the official DK1 tracker firmware specification.
* This implementation currently provides a skeleton for orientation estimation and HID communication.
* When creating tarballs on macOS, AppleDouble metadata files (e.g. `._CMakeLists.txt`) are automatically excluded by specifying `COPYFILE_DISABLE=1`:
  ```bash
  COPYFILE_DISABLE=1 tar -czf DK1Tracker-<version>.tar.gz DK1Tracker-<version>
  ```
* The example `dk1_dump` now supports a `--raw` flag to print raw report bytes before the parsed values.
