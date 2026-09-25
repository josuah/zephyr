# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(video_tcpserversink)

file(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})

include(${ZEPHYR_BASE}/samples/net/common/common.cmake)

#zephyr_code_relocate(LIBRARY subsys__net__ip FILTER "^\\.(noinit)\\.*" LOCATION PSRAM NOKEEP)
