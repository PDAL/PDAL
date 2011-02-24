#define BOOST_TEST_DYN_LINK

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>

#include "libpc/LiblasReader.hpp"

using namespace libpc;

BOOST_AUTO_TEST_SUITE(LiblasReaderTest)

BOOST_AUTO_TEST_CASE(test_1)
{
    std::istream* ifs = Utils::openFile("test/data/1.2-with-color.las");
    
    LiblasReader reader(*ifs);

    Utils::closeFile(ifs);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
