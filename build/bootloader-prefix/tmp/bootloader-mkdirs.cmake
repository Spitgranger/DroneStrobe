# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/richard/Documents/Projects/esp-idf/components/bootloader/subproject"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/tmp"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/src/bootloader-stamp"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/src"
  "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/richard/Documents/Projects/esp-idf/prototype0/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
