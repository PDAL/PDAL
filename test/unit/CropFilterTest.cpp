#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/FauxReader.hpp"
#include "libpc/FauxWriter.hpp"
#include "libpc/CropFilter.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(CropFilterTest)

BOOST_AUTO_TEST_CASE(test_crop)
{
    Bounds<double> srcBounds(0.0, 0.0, 0.0, 100.0, 100.0, 100.0);

    // crop tjhe window to 1/8th the size
    Bounds<double> dstBounds(0.0, 0.0, 0.0, 50.0, 50.0, 50.0);
    
    FauxReader reader(srcBounds, 1000, FauxReader::Random);

    CropFilter filter(reader, dstBounds);

    FauxWriter writer(filter);

    boost::uint64_t numWritten = writer.write(1000);

    // 1000 * 1/8 = 125, plus or minus 10%
    BOOST_CHECK(Utils::compare_approx<double>(numWritten, 125, 12.5));

    // test all the values to +/- 10%
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinX(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinY(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMinZ(), 0.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxX(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxY(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getMaxZ(), 50.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgX(), 25.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgY(), 25.0, 5.0));
    BOOST_CHECK(Utils::compare_approx<float>(writer.getAvgZ(), 25.0, 5.0));

    return;
}

BOOST_AUTO_TEST_SUITE_END()
