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

\`\`\`bash
cmake -S . -B build
cmake --build build
\`\`\`

## Important Notes
- The parser bit layout for packed motion samples needs verification against the official DK1 tracker firmware specification.
- This implementation currently provides a skeleton for orientation estimation and HID communication.
