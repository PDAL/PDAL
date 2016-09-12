# TEST support
#

if (WITH_TESTS)
  # Without this MS builds fail from attempting to handle too many args.
  #
  # This is fixed in trunk
  # https://code.google.com/p/googletest/source/detail?r=675
  # but for now, we need this fix
  # http://stackoverflow.com/questions/12558327/google-test-in-visual-studio-2012
  if (MSVC AND MSVC_VERSION EQUAL 1700)
    add_definitions(/D _VARIADIC_MAX=10)
  endif()

  include_directories(${ROOT_DIR}/vendor/gtest/include
      ${ROOT_DIR}/vendor/gtest)
endif()
