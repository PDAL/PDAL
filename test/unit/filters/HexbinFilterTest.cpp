/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include <boost/test/unit_test.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/HexBin.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(HexbinFilterTest)


#ifdef PDAL_HAVE_HEXER

BOOST_AUTO_TEST_CASE(HexbinFilterTest_test_1)
{


        pdal::Options options;
        
        pdal::Option filename("filename", Support::datapath("1.2-with-color.las"));

        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");
        // options.add(debug);
        // options.add(verbose);
        pdal::Option sample_size("sample_size",5000, "Number of samples to use when estimating hexagon edge size. Specify 0.0 for edge_size if you want to compute one.");
        pdal::Option threshold("threshold", 10, "Number of points necessary inside a hexagon to be considered full");
        pdal::Option edge_size("edge_size", 0.0, "The edge size of the hexagon to use in situations where you do not want to estimate based on a sample");
        pdal::Option x_dim("x_dim", "X", "dot-qualified name of X dimension to use");
        pdal::Option y_dim("y_dim", "Y", "dot-qualified name of Y dimension to use");
            
        options.add(filename);
        options.add(sample_size);
        options.add(threshold);
        options.add(edge_size);
        options.add(x_dim);
        options.add(y_dim);

        pdal::drivers::las::Reader reader(options);
        pdal::filters::HexBin hexbin(reader, options);
        

        hexbin.initialize();

        const pdal::Schema& schema = reader.getSchema();
        pdal::PointBuffer data(schema, reader.getNumPoints());
        

        pdal::StageSequentialIterator* iter = hexbin.createSequentialIterator(data);



        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 1065);

        pdal::filters::iterators::sequential::HexBin* b = static_cast<pdal::filters::iterators::sequential::HexBin*>(iter);


        delete iter;



    return;
}



#endif

BOOST_AUTO_TEST_SUITE_END()
