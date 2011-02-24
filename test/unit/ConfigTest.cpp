#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/libpc_defines.h"
#include "libpc/libpc_config.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(ConfigTest)

BOOST_AUTO_TEST_CASE(test_3rdparty_libs)
{
    bool liblas = IsLibLASEnabled();
    bool gdal = IsGDALEnabled();
    bool geotiff = IsLibGeoTIFFEnabled();
    bool laszip = IsLasZipEnabled();

#ifdef LIBPC_HAVE_LIBLAS
    BOOST_CHECK(liblas);
#else
    BOOST_CHECK(!liblas);
#endif

#ifdef LIBPC_HAVE_GDAL
    BOOST_CHECK(gdal);
#else
    BOOST_CHECK(!gdal);
#endif

#ifdef LIBPC_HAVE_LIBGEOTIFF
    BOOST_CHECK(geotiff);
#else
    BOOST_CHECK(!geotiff);
#endif

#ifdef LIBPC_HAVE_LASZIP
    BOOST_CHECK(laszip);
#else
    BOOST_CHECK(!laszip);
#endif

    return;
}

BOOST_AUTO_TEST_CASE(test_version)
{
    // just verify these functions can be called, don't worry about the values
    
    std::string version = GetVersionString();
    BOOST_CHECK(!version.empty());
    std::string fullVersion = GetFullVersionString();
    BOOST_CHECK(!fullVersion.empty());

    int major = GetVersionMajor();
    BOOST_CHECK(major >= 0);
    int minor = GetVersionMinor();
    BOOST_CHECK(minor >= 0);
    int patch = GetVersionPatch();
    BOOST_CHECK(patch >= 0);

    int bignum = GetVersionInteger();
    BOOST_CHECK(bignum > 0);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
