#
# Google Cloud Storage support
#


option(WITH_GCS
    "Build with OpenSSL and others for Google storage IO support" TRUE)
if (WITH_GCS)
    find_package(OpenSSL 1.1)
    if (OPENSSL_FOUND)
        set(ARBITER_OPENSSL TRUE)
    else ()
        message("OpenSSL NOT found - `export OPENSSL_ROOT_DIR=___`")
        message("Google storage IO will not be available")
        set(ARBITER_OPENSSL FALSE)
    endif ()
endif ()
