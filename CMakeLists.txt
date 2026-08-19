# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# NOTE: MINIMAL_BUILD is deliberately NOT set. The wsm_driver TOE backend works
# by intercepting lwIP's BSD socket entry points with -Wl,--wrap and by leaning
# on the VFS/newlib close() path; trimming the component graph is an easy way to
# drop a component those link-time hooks still expect to be present.
project(mqtt_WIZnet)
