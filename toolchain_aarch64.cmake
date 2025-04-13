# --- Setup ---
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TOOLCHAIN_ROOT /home/huy/ros2_ws/arm-gnu-toolchain-12.2.rel1-x86_64-aarch64-none-linux-gnu)

# --- Compilers ---
set(CMAKE_C_COMPILER ${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_ROOT}/bin/aarch64-none-linux-gnu-g++)

# --- Sysroot ---
set(CMAKE_SYSROOT /home/huy/mnt/usb_ext4)

# C/C++ flags
set(CMAKE_C_FLAGS "--sysroot=${CMAKE_SYSROOT} -I${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS "--sysroot=${CMAKE_SYSROOT} -I${CMAKE_SYSROOT}/usr/include/aarch64-linux-gnu -L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath,${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu" CACHE STRING "" FORCE)


# Thêm đường dẫn thư viện GCC vào CMAKE_LIBRARY_PATH
set(CMAKE_LIBRARY_PATH "${CMAKE_LIBRARY_PATH}:/home/huy/mnt/usb_ext4/usr/lib/gcc/aarch64-linux-gnu/11")

# --- Avoid linking wrong libstdc++ ---
# Prevent toolchain's lib64/libstdc++.a from being picked up
set(CMAKE_EXE_LINKER_FLAGS "-L${CMAKE_SYSROOT}/usr/lib/gcc/aarch64-linux-gnu/11 -Wl,-rpath,${CMAKE_SYSROOT}/usr/lib/gcc/aarch64-linux-gnu/11" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS}" CACHE STRING "" FORCE)
# Optional: for CMake >= 3.13
set(CMAKE_CXX_STANDARD_LIBRARIES "-L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu -lstdc++ -lm -lc")



set(CMAKE_IGNORE_PATH "${TOOLCHAIN_ROOT}/aarch64-none-linux-gnu/lib64")

# --- ROS search path ---
set(CMAKE_PREFIX_PATH "${CMAKE_SYSROOT}/opt/ros/rolling" CACHE STRING "" FORCE)


# --- Find paths ---
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
