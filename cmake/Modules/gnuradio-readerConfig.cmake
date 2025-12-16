find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_READER gnuradio-reader)

FIND_PATH(
    GR_READER_INCLUDE_DIRS
    NAMES gnuradio/reader/api.h
    HINTS $ENV{READER_DIR}/include
        ${PC_READER_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_READER_LIBRARIES
    NAMES gnuradio-reader
    HINTS $ENV{READER_DIR}/lib
        ${PC_READER_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-readerTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_READER DEFAULT_MSG GR_READER_LIBRARIES GR_READER_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_READER_LIBRARIES GR_READER_INCLUDE_DIRS)
