# Install script for directory: C:/Users/Olle/Documents/ncs/ncs/zephyr

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/Zephyr-Kernel")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/arch/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/lib/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/soc/arm/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/boards/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/ext/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/subsys/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/drivers/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/nrf/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/nffs/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/segger/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/mbedtls/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/mcuboot/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/mcumgr/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/tinycbor/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/modules/nrfxlib/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/kernel/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/cmake/flash/cmake_install.cmake")
  include("C:/Users/Olle/Documents/ncs/ncs/nrf/samples/bluetooth/peripheral_uart/build_nrf52810_pca10040/zephyr/cmake/reports/cmake_install.cmake")

endif()

