# Get the operating system information. Generally, CMake does a good job of
# this. Sometimes, though, it doesn't give enough information. These steps will
# distinguish between the UNIX variants. Otherwise, use the CMake variables
# such as WIN32 and APPLE and CYGWIN.
# Sets OS_IS_64BIT if the operating system is 64-bit.
# Sets LINUX if the operating system is Linux.
string(REGEX MATCH "Linux" OS_IS_LINUX ${CMAKE_SYSTEM_NAME})
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(OS_IS_64BIT TRUE)
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(OS_IS_64BIT FALSE)
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)
