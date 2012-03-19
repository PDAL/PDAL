/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/
#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <boost/test/unit_test.hpp>

#include <pdal/filters/Predicate.hpp>
#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>

#include <boost/scoped_ptr.hpp>

BOOST_AUTO_TEST_SUITE(PredicateFilterTest)

using namespace pdal;

BOOST_AUTO_TEST_CASE(PredicateFilterTest_test1)
{
    Bounds<double> bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Ramp);

    // keep all points where x less than 1.0
    const pdal::Option source("source", 
        // "X < 1.0"
        "import numpy as np\n"
        "def yow1(ins,outs):\n"
        "  X = ins['X']\n"
        "  Mask = np.less(X, 1.0)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
        );
    const pdal::Option module("module", "MyModule1");
    const pdal::Option function("function", "yow1");
    pdal::Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    pdal::filters::Predicate filter(reader, opts);
    BOOST_CHECK(filter.getDescription() == "Predicate Filter");
    pdal::drivers::faux::Writer writer(filter, Options::none());
    writer.initialize();

    boost::uint64_t numWritten = writer.write(1000);

    BOOST_CHECK(numWritten == 500);

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();

    BOOST_CHECK(Utils::compare_approx<double>(minX, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minY, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minZ, 0.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxX, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxY, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxZ, 1.0, 0.01));

    return;
}


BOOST_AUTO_TEST_CASE(PredicateFilterTest_test2)
{
    // same as above, but with 'Y >' instead of 'X <'

    Bounds<double> bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Ramp);

    const pdal::Option source("source", 
        // "Y > 1.0"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 1.0)\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
        );
    const pdal::Option module("module", "MyModule1");
    const pdal::Option function("function", "yow2");
    pdal::Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    pdal::filters::Predicate filter(reader, opts);
    BOOST_CHECK(filter.getDescription() == "Predicate Filter");
    pdal::drivers::faux::Writer writer(filter, Options::none());
    writer.initialize();

    boost::uint64_t numWritten = writer.write(1000);

    BOOST_CHECK(numWritten == 500);

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();

    BOOST_CHECK(Utils::compare_approx<double>(minX, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minY, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minZ, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxX, 2.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxY, 2.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxZ, 2.0, 0.01));

    return;
}


BOOST_AUTO_TEST_CASE(PredicateFilterTest_test3)
{
    // can we make a pipeline with TWO python filters in it?

    Bounds<double> bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Ramp);

    // keep all points where x less than 1.0
    const pdal::Option source1("source", 
        // "X < 1.0"
        "import numpy as np\n"
        "def yow1(ins,outs):\n"
        "  X = ins['X']\n"
        "  Mask = np.less(X, 1.0)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
        );
    const pdal::Option module1("module", "MyModule1");
    const pdal::Option function1("function", "yow1");
    pdal::Options opts1;
    opts1.add(source1);
    opts1.add(module1);
    opts1.add(function1);

    pdal::filters::Predicate filter1(reader, opts1);

    // keep all points where y greater than 0.5
    const pdal::Option source2("source", 
        // "Y > 0.5"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 0.5)\n"
        "  #print X\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
        );
    const pdal::Option module2("module", "MyModule2");
    const pdal::Option function2("function", "yow2");
    pdal::Options opts2;
    opts2.add(source2);
    opts2.add(module2);
    opts2.add(function2);

    pdal::filters::Predicate filter2(filter1, opts2);

    pdal::drivers::faux::Writer writer(filter2, Options::none());
    writer.initialize();

    boost::uint64_t numWritten = writer.write(1000);

    BOOST_CHECK(numWritten == 250);

    const double minX = writer.getMinX();
    const double minY = writer.getMinY();
    const double minZ = writer.getMinZ();
    const double maxX = writer.getMaxX();
    const double maxY = writer.getMaxY();
    const double maxZ = writer.getMaxZ();

    BOOST_CHECK(Utils::compare_approx<double>(minX, 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minY, 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(minZ, 0.5, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxX, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxY, 1.0, 0.01));
    BOOST_CHECK(Utils::compare_approx<double>(maxZ, 1.0, 0.01));

    return;
}


BOOST_AUTO_TEST_CASE(PredicateFilterTest_test4)
{
    // test the point counters in the Predicate's iterator

    Bounds<double> bounds(0.0, 0.0, 0.0, 2.0, 2.0, 2.0);
    pdal::drivers::faux::Reader reader(bounds, 1000, pdal::drivers::faux::Reader::Ramp);

    const pdal::Option source("source", 
        // "Y > 0.5"
        "import numpy as np\n"
        "def yow2(ins,outs):\n"
        "  Y = ins['Y']\n"
        "  Mask = np.greater(Y, 0.5)\n"
        "  #print Mask\n"
        "  outs['Mask'] = Mask\n"
        "  return True\n"
        );
    const pdal::Option module("module", "MyModule1");
    const pdal::Option function("function", "yow2");
    pdal::Options opts;
    opts.add(source);
    opts.add(module);
    opts.add(function);

    pdal::filters::Predicate filter(reader, opts);
    
    filter.initialize();

    const Schema& schema = filter.getSchema();
    PointBuffer data(schema, 1000);
    
    boost::scoped_ptr<pdal::StageSequentialIterator> iter(filter.createSequentialIterator(data));
    {
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 750);
    }
    
    pdal::filters::iterators::sequential::Predicate* iterator = static_cast<pdal::filters::iterators::sequential::Predicate*>(iter.get());

    const boost::uint64_t processed = iterator->getNumPointsProcessed();
    const boost::uint64_t passed = iterator->getNumPointsPassed();

    BOOST_CHECK(processed == 1000);
    BOOST_CHECK(passed == 750);

    return;
}

BOOST_AUTO_TEST_SUITE_END()
#endif
