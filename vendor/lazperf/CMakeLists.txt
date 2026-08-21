#
# Make sure we don't attempt to add a library more than once
#

file(GLOB SRCS
    *.cpp
    detail/*.cpp
)
set(LAZPERF_SRCS ${SRCS} PARENT_SCOPE)

