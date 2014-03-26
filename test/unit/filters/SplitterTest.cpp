/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <boost/cstdint.hpp>

#include <pdal/filters/Splitter.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(SplitterTest)

BOOST_AUTO_TEST_CASE(test_tile_filter)
{
    // create the reader
    pdal::drivers::las::Reader r(Support::datapath("1.2-with-color.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream

        // create the options
        pdal::Options o;
        pdal::Option length("length", 1000, "length");
        o.add(length);

        // create the tile filter and initialize
        pdal::filters::Splitter s(r, o);
        s.initialize();

        // get the bounds of the entire dataset
        pdal::Bounds<double> const& b = s.getPrevStage().getBounds();

        // verify bounds
        BOOST_CHECK_CLOSE(b.getMinimum(0), 635619.85, 0.05);
        BOOST_CHECK_CLOSE(b.getMaximum(0), 638982.55, 0.05);
        BOOST_CHECK_CLOSE(b.getMinimum(1), 848899.70, 0.05);
        BOOST_CHECK_CLOSE(b.getMaximum(1), 853535.43, 0.05);
        BOOST_CHECK_CLOSE(b.getMinimum(2), 406.59, 0.05);
        BOOST_CHECK_CLOSE(b.getMaximum(2), 586.38, 0.05);

        // create buffer and generate tiles
        PointBuffer buffer(s.getSchema(), s.getNumPoints());
        PointBuffer tiled_buffer(s.getSchema(), s.getNumPoints());
        s.generateTiles(buffer, tiled_buffer);

        // verify the correct number of tiles computed at the given tile size
        BOOST_CHECK(s.getTileCount() == 24);

        // tiled buffer is the same size as the input buffer
        BOOST_CHECK(buffer.getNumPoints() == tiled_buffer.getNumPoints());

        // check block point counts - how many points are allocated within each tile
        BOOST_CHECK(s.getTile(0).getNumPoints() ==  1);  // lower left xy: 635000, 848000
        BOOST_CHECK(s.getTile(1).getNumPoints() ==  3);  // lower left xy: 636000, 848000
        BOOST_CHECK(s.getTile(2).getNumPoints() ==  4);  // lower left xy: 637000, 848000
        BOOST_CHECK(s.getTile(3).getNumPoints() ==  4);  // lower left xy: 638000, 848000
        BOOST_CHECK(s.getTile(4).getNumPoints() == 25);  // lower left xy: 635000, 849000
        BOOST_CHECK(s.getTile(5).getNumPoints() == 57);  // lower left xy: 636000, 849000
        BOOST_CHECK(s.getTile(6).getNumPoints() == 70);  // lower left xy: 637000, 849000
        BOOST_CHECK(s.getTile(7).getNumPoints() == 58);  // lower left xy: 638000, 849000
        BOOST_CHECK(s.getTile(8).getNumPoints() == 24);  // lower left xy: 635000, 850000
        BOOST_CHECK(s.getTile(9).getNumPoints() == 78);  // lower left xy: 636000, 850000
        BOOST_CHECK(s.getTile(10).getNumPoints() == 87); // lower left xy: 637000, 850000
        BOOST_CHECK(s.getTile(11).getNumPoints() == 67); // lower left xy: 638000, 850000
        BOOST_CHECK(s.getTile(12).getNumPoints() == 25); // lower left xy: 635000, 851000
        BOOST_CHECK(s.getTile(13).getNumPoints() == 71); // lower left xy: 636000, 851000
        BOOST_CHECK(s.getTile(14).getNumPoints() == 72); // lower left xy: 637000, 851000
        BOOST_CHECK(s.getTile(15).getNumPoints() == 62); // lower left xy: 638000, 851000
        BOOST_CHECK(s.getTile(16).getNumPoints() == 24); // lower left xy: 635000, 852000
        BOOST_CHECK(s.getTile(17).getNumPoints() == 74); // lower left xy: 636000, 852000
        BOOST_CHECK(s.getTile(18).getNumPoints() == 62); // lower left xy: 637000, 852000
        BOOST_CHECK(s.getTile(19).getNumPoints() == 70); // lower left xy: 638000, 852000
        BOOST_CHECK(s.getTile(20).getNumPoints() == 10); // lower left xy: 635000, 853000
        BOOST_CHECK(s.getTile(21).getNumPoints() == 46); // lower left xy: 636000, 853000
        BOOST_CHECK(s.getTile(22).getNumPoints() == 34); // lower left xy: 637000, 853000
        BOOST_CHECK(s.getTile(23).getNumPoints() == 37); // lower left xy: 638000, 853000
    }

    return;
}

BOOST_AUTO_TEST_SUITE_END()

