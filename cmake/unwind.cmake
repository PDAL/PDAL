#
# Unwind
#

set(FPHSA_NAME_MISMATCHED 1) # Suppress warnings, see https://cmake.org/cmake/help/v3.17/module/FindPackageHandleStandardArgs.html
find_package(Libunwind QUIET)
set(FPHSA_NAME_MISMATCHED 0)

