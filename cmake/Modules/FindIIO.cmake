INCLUDE(FindPkgConfig)
PKG_CHECK_MODULES(PC_IIO "libiio")

FIND_PATH(IIO_INCLUDE_DIRS
    NAMES iio.h
    HINTS ${PC_IIO_INCLUDE_DIR}
    ${CMAKE_INSTALL_PREFIX}/include
    PATHS
    /usr/local/include
    /usr/include
)

FIND_LIBRARY(IIO_LIBRARIES
    NAMES iio libiio
    HINTS ${PC_IIO_LIBDIR}
    ${CMAKE_INSTALL_PREFIX}/lib
    ${CMAKE_INSTALL_PREFIX}/lib64
    PATHS
    ${IIO_INCLUDE_DIRS}/../lib
    /usr/local/lib
    /usr/lib
)

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(IIO DEFAULT_MSG IIO_LIBRARIES IIO_INCLUDE_DIRS)
MARK_AS_ADVANCED(IIO_LIBRARIES IIO_INCLUDE_DIRS)
