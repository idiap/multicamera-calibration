# - Find GFlags library
# This module finds GFlags library and its header files.
# This code sets the following variables:
#
#  GFLAGS_FOUND         - has GFlags library been found?
#  GFLAGS_LIBRARIES     - names of the GFlags libraries
#  GFLAGS_INCLUDE_PATH  - path to the GFlags includes
#
# and provides the following defines:
#
#  __GFLAGS_FOUND__
#

if (NOT GFLAGS_FOUND)

    set(GFLAGS_FOUND TRUE)
    
    set(GFLAGS_LIBRARYNAME "gflags")
    
    # find GFlags library
    find_library(GFLAGS_LIBRARY NAMES ${GFLAGS_LIBRARYNAME})
    
    if (NOT GFLAGS_LIBRARY)
        set(GFLAGS_FOUND FALSE)
        set(GFLAGS_ERROR_REASON "${GFLAGS_ERROR_REASON} Libraries not found.")
    else()
        get_filename_component(GFLAGS_LIBRARY_PATH ${GFLAGS_LIBRARY} PATH CACHE)
        set(GFLAGS_LIBRARIES ${GFLAGS_LIBRARYNAME})
        message(STATUS "GFlags libraries: ${GFLAGS_LIBRARIES}")
        message(STATUS "GFlags library path: ${GFLAGS_LIBRARY_PATH}")
    endif()
    
    # find GFlags headers
    find_path(GFLAGS_INCLUDE_PATH NAMES gflags/gflags.h)
    
    if (NOT GFLAGS_INCLUDE_PATH)
        set(GFLAGS_FOUND FALSE)
        set(GFLAGS_ERROR_REASON "${GFLAGS_ERROR_REASON} Includes not found.")
    else()
        message(STATUS "GFlags includes: ${GFLAGS_INCLUDE_PATH}")
    endif()

    if (NOT GFLAGS_FOUND)
        if(gflags_FIND_REQUIRED)
            message(SEND_ERROR "Unable to find required GFlags.\n${GFLAGS_ERROR_REASON}")
        else()
            message(STATUS "Unable to find GFlags: ${GFLAGS_ERROR_REASON}")
        endif()
    else()
        ADD_DEFINITIONS(-D__GFLAGS_FOUND__)
        include_directories(${GFLAGS_INCLUDE_PATH})
        link_directories   (${GFLAGS_LIBRARY_PATH})
    endif()

endif()
