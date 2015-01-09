#
# To reduce typing.
#
set(CDIR "${CMAKE_CURRENT_LIST_DIR}")

#
# This must be first.
#
include(${CDIR}/directories.cmake)

#
# This must come before macros, but I don't understand why the policies
# apply to the macros rather than the invocation of the macros.
#
include(${CDIR}/policies.cmake NO_POLICY_SCOPE)
include(${CDIR}/macros.cmake)
include(${CDIR}/libraries.cmake)
include(${CDIR}/compiler_options.cmake)
include(${CDIR}/modules.cmake)
