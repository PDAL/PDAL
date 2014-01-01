# - Find msgpack
# Find the native Message Pack headers and libraries.
#
#  MsgPack_INCLUDE_DIRS - where to find msgpack.hpp, etc.
#  MsgPack_LIBRARIES    - List of libraries when using MsgPack.
#  MsgPack_FOUND        - True if MsgPack found.

# Taken from https://github.com/qdot/kinect-recorder/blob/master/FindMsgPack.cmake

# Look for the header file.
FIND_PATH(MsgPack_INCLUDE_DIR NAMES msgpack.hpp)

# Look for the library.
FIND_LIBRARY(MsgPack_LIBRARY NAMES msgpack libmsgpack)

# handle the QUIETLY and REQUIRED arguments and set MsgPack_FOUND to TRUE if 
# all listed variables are TRUE
INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MsgPack DEFAULT_MSG MsgPack_LIBRARY MsgPack_INCLUDE_DIR)

# Copy the results to the output variables.
IF(MSGPACK_FOUND)
  SET(MsgPack_LIBRARIES ${MsgPack_LIBRARY})
  SET(MsgPack_INCLUDE_DIRS ${MsgPack_INCLUDE_DIR})
ELSE(MSGPACK_FOUND)
  SET(MsgPack_LIBRARIES)
  SET(MsgPack_INCLUDE_DIRS)
ENDIF(MSGPACK_FOUND)

MARK_AS_ADVANCED(MsgPack_INCLUDE_DIR MsgPack_LIBRARY)