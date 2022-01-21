if(NOT PKG_CONFIG_FOUND)
    INCLUDE(FindPkgConfig)
endif()
PKG_CHECK_MODULES(PC_RWT rwt)

FIND_PATH(
    RWT_INCLUDE_DIRS
    NAMES rwt/api.h
    HINTS $ENV{RWT_DIR}/include
        ${PC_RWT_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    RWT_LIBRARIES
    NAMES gnuradio-rwt
    HINTS $ENV{RWT_DIR}/lib
        ${PC_RWT_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/rwtTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(RWT DEFAULT_MSG RWT_LIBRARIES RWT_INCLUDE_DIRS)
MARK_AS_ADVANCED(RWT_LIBRARIES RWT_INCLUDE_DIRS)
