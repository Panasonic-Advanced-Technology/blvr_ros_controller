set(MODBUS_ROOT_DIR
      "${MODBUS_ROOT_DIR}"
    CACHE
    PATH
      "Root directory to search for libmodbus")

find_path(MODBUS_INCLUDE_DIR
  NAMES
    modbus.h
  HINTS
    "${MODBUS_ROOT_DIR}"
  PATH_SUFFIXES
    include
  PATHS
    /usr
    /usr/local
    /usr/include/modbus
    /usr/local/include/modbus
  )

find_library(MODBUS_LIBRARY
  NAMES
    libmodbus.so
    modbus
  PATH_SUFFIXES
    lib
  PATH
    /usr
    /usr/local
  )

message(STATUS "Here" ${MODBUS_LIBRARY})

include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(Modbus
    DEFAULT_MSG
    MODBUS_LIBRARY
    MODBUS_INCLUDE_DIR
  )

if(MODBUS_FOUND)
  set(MODBUS_LIBRARIES ${MODBUS_LIBRARY})
  set(MODBUS_INCLUDE_DIRS ${MODBUS_INCLUDE_DIR})
else()
  set(MODBUS_LIBRARIES)
  set(MODBUS_INCLUDE_DIRS)
endif()

mark_as_advanced(MODBUS_LIBRARY MODBUS_INCLUDE_DIR)

