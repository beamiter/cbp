# Install script for directory: /home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/example

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/abs_normal/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/atomic/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/deprecated/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/general/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/get_started/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/multi_thread/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/optimize/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/print_for/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/sparse/cmake_install.cmake")
  include("/home/idriver/sehs/src/kernel/ivplanning/ivpathplanner/src/avoidObsModel/cbp/cppad.git/build/example/utility/cmake_install.cmake")

endif()

