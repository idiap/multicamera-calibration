# - Find GLog library
# This module finds GLog library and its header files.
# This code sets the following variables:
#
#  GLOG_FOUND         - has GLog library been found?
#  GLOG_LIBRARIES     - names of the GLog libraries
#  GLOG_INCLUDE_PATH  - path to the GLog includes
#
# and provides the following defines:
#
#  __GLOG_FOUND__
#

if (NOT GLOG_FOUND)

    set(GLOG_FOUND TRUE)
    
    set(GLOG_LIBRARYNAME "glog")
    
    # find GLog library
    find_library(GLOG_LIBRARY NAMES ${GLOG_LIBRARYNAME})
    
    if (NOT GLOG_LIBRARY)
        set(GLOG_FOUND FALSE)
        set(GLOG_ERROR_REASON "${GLOG_ERROR_REASON} Libraries not found.")
    else()
        get_filename_component(GLOG_LIBRARY_PATH ${GLOG_LIBRARY} PATH CACHE)
        set(GLOG_LIBRARIES ${GLOG_LIBRARYNAME})
        message(STATUS "GLog libraries: ${GLOG_LIBRARIES}")
        message(STATUS "GLog library path: ${GLOG_LIBRARY_PATH}")
    endif()
    
    # find GLog headers
    find_path(GLOG_INCLUDE_PATH NAMES gflags/gflags.h)
    
    if (NOT GLOG_INCLUDE_PATH)
        set(GLOG_FOUND FALSE)
        set(GLOG_ERROR_REASON "${GLOG_ERROR_REASON} Includes not found.")
    else()
        message(STATUS "GLog includes: ${GLOG_INCLUDE_PATH}")
    endif()

    if (NOT GLOG_FOUND)
        if(glog_FIND_REQUIRED)
            message(SEND_ERROR "Unable to find required GLog.\n${GLOG_ERROR_REASON}")
        else()
            message(STATUS "Unable to find GLog: ${GLOG_ERROR_REASON}")
        endif()
    else()
        ADD_DEFINITIONS(-D__GLOG_FOUND__)
        include_directories(${GLOG_INCLUDE_PATH})
        link_directories   (${GLOG_LIBRARY_PATH})
    endif()

endif()
