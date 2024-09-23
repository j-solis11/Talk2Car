# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/perso/.espressif/frameworks/esp-idf-v5.2/components/bootloader/subproject"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/tmp"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/src"
  "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/perso/OneDrive/Documents/GitHub/Team05-Shib-Solis-Zhao/quest-4/code/buggy/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
