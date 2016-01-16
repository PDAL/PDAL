#
# TEST support
#

# Without this MS builds fail from attempting to handle too many args.
#
# This is fixed in trunk
# https://code.google.com/p/googletest/source/detail?r=675
# but for now, we need this fix
# http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-201
if (MSVC AND MSVC_VERSION EQUAL 1700)
    add_definitions(/D _VARIADIC_MAX=10)
endif()

include_directories(vendor/gtest-1.7.0/include vendor/gtest-1.7.0)
