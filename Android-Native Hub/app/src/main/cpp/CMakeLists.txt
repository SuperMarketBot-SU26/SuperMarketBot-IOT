# SmartMarketBot - CMakeLists.txt for Native C++ (NDK)

cmake_minimum_required(VERSION 3.18)
project(smartmarketbot_slam)

# C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Android specific
set(ANDROID_ABI "arm64-v8a")
set(ANDROID_NATIVE_API_LEVEL 24)
set(ANDROID_STL c++_shared)

# Find packages
find_package(Eigen3 QUIET)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Source files
# udp_socket.cpp không còn được dùng — project chuyển sang WebSocket JSON.
# Giữ file trong repo để tham khảo, không include vào build.
set(UTILITY_SOURCES
)

# Create static library
add_library(smartmarketbot_slam STATIC
    ${UTILITY_SOURCES}
)

# Link libraries
target_link_libraries(smartmarketbot_slam
    log
    android
)

# Optional: Eigen
if(Eigen3_FOUND)
    target_include_directories(smartmarketbot_slam PUBLIC
        ${EIGEN3_INCLUDE_DIR}
    )
endif()

# Add JNI wrapper
add_library(native_slam SHARED
    src/jni_wrapper.cpp
)

target_link_libraries(native_slam
    smartmarketbot_slam
    log
)
