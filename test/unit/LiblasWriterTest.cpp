#ifdef _MSC_VER
#define BOOST_TEST_DYN_LINK
#endif

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/FauxReader.hpp"
#include "libpc/src/drivers/liblas/writer.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(LiblasWriterTest)

BOOST_AUTO_TEST_CASE(test_1)
{
    // remove file from earlier run, if needed
    Utils::deleteFile("temp.las");

    Bounds<double> bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    FauxReader reader(bounds, 1000, FauxReader::Constant);

    std::ostream* ofs = Utils::createFile("temp.las");

    LiblasWriter writer(reader, *ofs);

    Utils::closeFile(ofs);

    Utils::deleteFile("temp.las");

    return;
}

BOOST_AUTO_TEST_SUITE_END()
