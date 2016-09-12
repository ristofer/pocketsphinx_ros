# Find PocketSphinx on the development system.
# This module finds if PocketSphinx is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  PocketSphinx_VERSION           - Version of PocketSphinx
#  PocketSphinx_LIBRARIES         - Path to the PocketSphinx library
#  PocketSphinx_FOUND             - Boolean that indicates if the package 
#                                   was found
#  PocketSphinx_INCLUDE_DIRS      - Path to where pocketsphinx.h is found
#  PocketSphinx_MODELDIR          - Path to where preinstalled models would 
#                                   be found
#
###############################################################################

find_package(PkgConfig QUIET)
pkg_check_modules(PC_POCKETSPHINX pocketsphinx)

find_path(PocketSphinx_INCLUDE_DIRS pocketsphinx.h
    HINTS ${PC_POCKETSPHINX_INCLUDEDIR} ${PC_POCKETSPHINX_INCLUDE_DIRS}
    PATH_SUFFIXES pocketsphinx)
find_library(PocketSphinx_LIBRARIES pocketsphinx
    HINTS ${PC_POCKETSPHINX_LIBRARY_DIRS} ${PC_POCKETSPHINX_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(PocketSphinx DEFAULT_MSG
    PocketSphinx_LIBRARIES PocketSphinx_INCLUDE_DIRS)

list(APPEND PocketSphinx_INCLUDE_DIRS ${PC_POCKETSPHINX_INCLUDE_DIRS})

mark_as_advanced(PocketSphinx_INCLUDE_DIRS POCKETSPHINX_LIBRARIES)

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} pocketsphinx --variable=modeldir
                OUTPUT_VARIABLE PocketSphinx_MODELDIR
                OUTPUT_STRIP_TRAILING_WHITESPACE)

set(PocketSphinx_VERSION ${PC_POCKETSPHINX_VERSION})

if(POCKETSPHINX_FOUND)
  message(STATUS "PocketSphinx found (include: ${PocketSphinx_INCLUDE_DIRS})")
endif(POCKETSPHINX_FOUND)
set(PocketSphinx_FOUND ${POCKETSPHINX_FOUND})

# Debug messages
# MESSAGE(STATUS "PocketSphinx libraries: ${PocketSphinx_LIBRARIES}")
# MESSAGE(STATUS "PocketSphinx include dir: ${PocketSphinx_INCLUDE_DIRS}")
# MESSAGE(STATUS "PocketSphinx model dir: ${PocketSphinx_MODELDIR}")
# MESSAGE(STATUS "PocketSphinx version: ${PocketSphinx_VERSION}")
# MESSAGE(STATUS "PocketSphinx found: ${PocketSphinx_FOUND}")