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
    // just verify it is a string, don't worry about the contents
    
    std::string version = GetVersionString();
    std::string fullVersion = GetFullVersionString();

    int major = GetVersionMajor();
    int minor = GetVersionMinor();
    int patch = GetVersionPatch();

    int x = GetVersionInteger();

    return;
}

BOOST_AUTO_TEST_SUITE_END()
