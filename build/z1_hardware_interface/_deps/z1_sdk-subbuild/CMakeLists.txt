# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.22.1)

# We name the project and the target for the ExternalProject_Add() call
# to something that will highlight to the user what we are working on if
# something goes wrong and an error message is produced.

project(z1_sdk-populate NONE)



include(ExternalProject)
ExternalProject_Add(z1_sdk-populate
                     "UPDATE_DISCONNECTED" "False" "URL" "https://github.com/unitreerobotics/z1_sdk/archive/refs/heads/master.zip"
                    SOURCE_DIR          "/home/wangkun/z1_ros2_ws/build/z1_hardware_interface/_deps/z1_sdk-src"
                    BINARY_DIR          "/home/wangkun/z1_ros2_ws/build/z1_hardware_interface/_deps/z1_sdk-build"
                    CONFIGURE_COMMAND   ""
                    BUILD_COMMAND       ""
                    INSTALL_COMMAND     ""
                    TEST_COMMAND        ""
                    USES_TERMINAL_DOWNLOAD  YES
                    USES_TERMINAL_UPDATE    YES
)


