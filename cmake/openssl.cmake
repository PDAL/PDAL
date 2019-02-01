find_package(OpenSSL 1.0.1)
if (OPENSSL_FOUND)
    set(ARBITER_OPENSSL TRUE)
else()
    # For me this is /usr/local/opt/openssl\@1.1
    message("OpenSSL NOT found - `export OPENSSL_ROOT_DIR=___`")
    message("Google storage IO will not be available")
endif()