# Find SphinxBase on the development system.
# This module finds if SphinxBase is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  SphinxBase_VERSION             - Version of SphinxBase
#  SphinxBase_LIBRARIES           - Path to the SphinxBase library
#  SphinxBase_FOUND               - Boolean that indicates if the package 
#                                   was found
#  SphinxAD_LIBRARIES			  - Path to the SphinxAD library
#  SphinxBase_INCLUDE_DIRS        - Path to where sphinxbase.h is found
#
###############################################################################

find_package(PkgConfig QUIET)

pkg_check_modules(PC_SPHINXBASE sphinxbase)

find_path(SphinxBase_INCLUDE_DIRS cmd_ln.h
    HINTS ${PC_SPHINXBASE_INCLUDEDIR} ${PC_SPHINXBASE_INCLUDE_DIRS})
find_library(SphinxBase_LIBRARIES sphinxbase
    HINTS ${PC_SPHINXBASE_LIBRARY_DIRS} ${PC_SPHINXBASE_LIBDIR})
find_library(SphinxAD_LIBRARIES sphinxad
    HINTS ${PC_SPHINXBASE_LIBRARY_DIRS} ${PC_SPHINXBASE_LIBDIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SphinxBase DEFAULT_MSG
    SphinxBase_LIBRARIES SphinxBase_INCLUDE_DIRS)

list(APPEND SphinxBase_INCLUDE_DIRS ${PC_SPHINXBASE_INCLUDE_DIRS})

set(SphinxBase_VERSION ${PC_SPHINXBASE_VERSION})

mark_as_advanced(SphinxBase_INCLUDE_DIRS SphinxBase_LIBRARIES SphinxBase_LIBRARIES)

if(SPHINXBASE_FOUND)
  message(STATUS "SphinxBase found (include: ${SphinxBase_INCLUDE_DIRS})")
endif(SPHINXBASE_FOUND)
set(SphinxBase_FOUND ${SPHINXBASE_FOUND})

# Debug messages
# MESSAGE(STATUS "SphinxBase libraries: ${SphinxBase_LIBRARIES}")
# MESSAGE(STATUS "SphinxAD include dir: ${SphinxAD_LIBRARIES}")
# MESSAGE(STATUS "SphinxBase model dir: ${SphinxBase_INCLUDE_DIRS}")
# MESSAGE(STATUS "SphinxBase version: ${SphinxBase_VERSION}")
# MESSAGE(STATUS "SphinxBase found: ${SphinxBase_FOUND}")
