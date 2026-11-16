# SPDX-License-Identifier: Apache-2.0

# Plain CMakeLists.txt built with: cmake -B build -S . -G Ninja && cmake --build build
# But in practice, west build also seems to work
# This is just used for experimenting and debugging as an alternative to `bl616_lp_fw.bin` that
# provides the expected features.

cmake_minimum_required(VERSION 3.28.0)

find_package(Zephyr-sdk COMPONENTS LIST)

project(lpfw)
enable_language(C)
enable_language(ASM)

set(CMAKE_SYSTEM_PROCESSOR rv32imacf_zicsr_zifencei_xtheadba_xtheadbb_xtheadbs_xtheadcmo)
set(TOOLCHAIN_PREFIX ${ZEPHYR_SDK_INSTALL_DIR}/gnu/riscv64-zephyr-elf/bin/riscv64-zephyr-elf-)
set(CMAKE_C_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_PREFIX}gcc)
set(CMAKE_C_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)

add_executable(lpfw.elf main.c start.S)
target_compile_options(lpfw.elf PUBLIC -specs=picolibc.specs)
target_compile_options(lpfw.elf PUBLIC -ffunction-sections)
target_compile_options(lpfw.elf PUBLIC -fdata-sections)
target_link_options(lpfw.elf PUBLIC -T ${CMAKE_CURRENT_LIST_DIR}/linker.ld)
target_link_options(lpfw.elf PUBLIC -nostdlib)
target_link_options(lpfw.elf PUBLIC -Wl,--gc-sections)

add_custom_target(lpfw_bin
  ALL
  BYPRODUCTS lpfw.bin
  DEPENDS lpfw.elf
  COMMAND ${TOOLCHAIN_PREFIX}objcopy -O binary lpfw.elf lpfw.bin
)

add_dependencies(lpfw_bin lpfw.elf)
