#
# OpenSSL support
#
option(WITH_OPENSSL
    "Build with OpenSSL for Google storage IO support" TRUE)
if (WITH_OPENSSL)
    find_package(OpenSSL 1.1)
    if (OPENSSL_FOUND)
        set(ARBITER_OPENSSL TRUE)
    else ()
        message("OpenSSL NOT found - `export OPENSSL_ROOT_DIR=___`")
        message("Google storage IO will not be available")
    endif ()
endif ()
