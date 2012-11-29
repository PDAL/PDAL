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

#include <boost/test/unit_test.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Colorization.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(ColorizationFilterTest)


#ifdef PDAL_HAVE_GDAL


BOOST_AUTO_TEST_CASE(ColorizationFilterTest_test_1)
{

    {

        pdal::drivers::las::Reader reader(Support::datapath("autzen-point-format-3.las"));

        pdal::Options options;

        pdal::Option red("dimension", "Red", "");
        pdal::Option b0("band",1, "");
        pdal::Option s0("scale", 1.0f, "scale factor for this dimension");
        pdal::Options redO;
        redO.add(b0);
        redO.add(s0);
        red.setOptions(redO);

        pdal::Option green("dimension", "Green", "");
        pdal::Option b1("band",2, "");
        pdal::Option s1("scale", 1.0f, "scale factor for this dimension");
        pdal::Options greenO;
        greenO.add(b1);
        greenO.add(s1);
        green.setOptions(greenO);

        pdal::Option blue("dimension", "Blue", "");
        pdal::Option b2("band",3, "");
        pdal::Option s2("scale", 255.0f, "scale factor for this dimension");
        pdal::Options blueO;
        blueO.add(b2);
        blueO.add(s2);
        blue.setOptions(blueO);

        pdal::Option datasource("raster", Support::datapath("autzen.jpg"), "raster to read");
        // pdal::Option verbose("verbose", 7, "");
        // pdal::Option debug("debug", true, "");

        pdal::Options reader_options;
        reader_options.add(red);
        reader_options.add(green);
        reader_options.add(blue);
        reader_options.add(datasource);
        // reader_options.add(debug);
        // reader_options.add(verbose);

        pdal::filters::Colorization filter(reader, reader_options);

        filter.initialize();

        const pdal::Schema& schema = filter.getSchema();
        pdal::PointBuffer data(schema, 1);

        pdal::StageSequentialIterator* iter = filter.createSequentialIterator(data);
        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK(numRead == 1);
        delete iter;

        const pdal::Schema& s = data.getSchema();

        pdal::Dimension const& dimRed = s.getDimension("Red");
        pdal::Dimension const& dimGreen = s.getDimension("Green");
        pdal::Dimension const& dimBlue = s.getDimension("Blue");

        boost::uint16_t r = data.getField<boost::uint16_t>(dimRed, 0);
        boost::uint16_t g = data.getField<boost::uint16_t>(dimGreen, 0);
        boost::uint16_t b = data.getField<boost::uint16_t>(dimBlue, 0);

// GDAL's JPEG driver was updated in 1.10 to use the libjpeg 
// capability of computing fast level 2, 4, and 8 overviews. This 
// means the results are numerically, if not visually, different
// than before.
#if ((GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 10))
        BOOST_CHECK_EQUAL(r, 195u);
        BOOST_CHECK_EQUAL(g, 176u);
        BOOST_CHECK_EQUAL(b, 36720u); // We scaled this up to 16bit by multiplying by 255

#else
        BOOST_CHECK_EQUAL(r, 210u);
        BOOST_CHECK_EQUAL(g, 205u);
        BOOST_CHECK_EQUAL(b, 47175u); // We scaled this up to 16bit by multiplying by 255

#endif

    }


    return;
}

#endif

BOOST_AUTO_TEST_SUITE_END()
