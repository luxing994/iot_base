# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# 联网由本项目 main/src/app/net_manager.c 实现，不再使用 ESP-IDF 示例组件 protocol_examples_common

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(iot_base)
