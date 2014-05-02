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
#include <boost/cstdint.hpp>

#include <pdal/filters/Chipper.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/Options.hpp>

#include "Support.hpp"

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(ChipperTest)


BOOST_AUTO_TEST_CASE(test_construction)
{
    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));

    {
        // need to scope the writer, so that's it dtor can use the stream

        pdal::Options options;
        pdal::Option capacity("capacity", 15, "capacity");
        options.add(capacity);

        pdal::filters::Chipper chipper(options);
        chipper.setInput(&reader);
        chipper.initialize();
        PointBuffer buffer(chipper.getSchema(), 15);
        const boost::uint64_t num_points = reader.getNumPoints();

        chipper.Chip(buffer);
        boost::uint32_t num_blocks = chipper.GetBlockCount();
        BOOST_CHECK(num_points == 1065);
        BOOST_CHECK(num_blocks==71);


        pdal::Bounds<double> const& bounds = chipper.GetBlock(0).GetBounds();


        std::vector< Range<double> > ranges = bounds.dimensions();

        pdal::Range<double> x = ranges[0];
        pdal::Range<double> y = ranges[1];

        BOOST_CHECK_CLOSE(x.getMinimum(), 635674.0, 0.05);
        BOOST_CHECK_CLOSE(x.getMaximum(), 635994.0, 0.05);
        BOOST_CHECK_CLOSE(y.getMinimum(), 848992.0, 0.05);
        BOOST_CHECK_CLOSE(y.getMaximum(), 849427.0, 0.05);

        std::vector<boost::uint32_t> ids = chipper.GetBlock(70).GetIDs();

        BOOST_CHECK(ids.size() == 15);
        BOOST_CHECK(ids[14] == 1050);


        //
        // std::cout << buffer.getField<boost::int32_t>(0, 0) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 0) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 0) << std::endl;
        //
        // std::cout << buffer.getField<boost::int32_t>(0, 1) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 1) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 1) << std::endl;
        //
        // std::cout << buffer.getField<boost::int32_t>(0, 2) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(1, 2) << std::endl;
        // std::cout << buffer.getField<boost::int32_t>(2, 2) << std::endl;

        // // Check X's of first three points in block 20
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimX, 0), 63763550);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimX, 1), 63765279);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimX, 2), 63771207);
        // 
        // // Check Y's of first three points in block 20
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimY, 0), 84992418);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimY, 1), 85005705);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimY, 2), 85005840);
        // 
        // // Check Z's of first three points in block 20
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimZ, 0), 42664);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimZ, 1), 43579);
        // BOOST_CHECK_EQUAL(buffer.getField<boost::int32_t>(dimZ, 2), 42651);

    }
}



BOOST_AUTO_TEST_CASE(test_ordering)
{
    std::string candidate_filename(Support::datapath("autzen-utm.las"));
    std::string source_filename(Support::datapath("autzen-utm-chipped-25.las"));
    
    pdal::Options options;
    Option filename("filename", source_filename, "");
    options.add(filename);

    Option capacity("capacity", 25,"capacity");
    options.add(capacity);
    
    pdal::drivers::las::Reader candidate_reader(options);
    pdal::filters::Cache cache(options);
    cache.setInput(&candidate_reader);
    pdal::filters::Chipper chipper(options);
    chipper.setInput(&cache);
    chipper.initialize();
    
    Option& query = options.getOptionByRef("filename");
    query.setValue<std::string>(source_filename);

    pdal::drivers::las::Reader source_reader(options);
    source_reader.initialize();
    
    BOOST_CHECK_EQUAL(chipper.getNumPoints(), source_reader.getNumPoints());
    
    pdal::PointBuffer candidate(chipper.getSchema(), chipper.getNumPoints());
    pdal::PointBuffer patch(chipper.getSchema(), chipper.getNumPoints());
    
    pdal::StageSequentialIterator* iter_c = chipper.createSequentialIterator(patch);
    boost::uint64_t numRead(0);
    
    while (true)
    {
        numRead = iter_c->read(patch);
        if (! numRead)
            break;
        candidate.copyPointsFast(candidate.getNumPoints(), 0, patch, patch.getNumPoints());
        candidate.setNumPoints(candidate.getNumPoints() + patch.getNumPoints());
    }
    BOOST_CHECK_EQUAL(candidate.getNumPoints(), chipper.getNumPoints());

    pdal::PointBuffer source(source_reader.getSchema(), source_reader.getNumPoints());
    
    pdal::StageSequentialIterator* iter_s = source_reader.createSequentialIterator(source);
    numRead = iter_s->read(source);
    BOOST_CHECK_EQUAL(numRead, source_reader.getNumPoints());
    
    
    
    pdal::Schema const& cs = candidate.getSchema();
    pdal::Schema const& ss = source.getSchema();
    
    pdal::Dimension const& sdimX = ss.getDimension("X");
    pdal::Dimension const& sdimY = ss.getDimension("Y");
    pdal::Dimension const& sdimZ = ss.getDimension("Z");
    pdal::Dimension const& sdimIntensity = ss.getDimension("Intensity");
    pdal::Dimension const& sdimRed = ss.getDimension("Red");
    pdal::Dimension const& sdimGreen = ss.getDimension("Green");
    pdal::Dimension const& sdimBlue = ss.getDimension("Blue");

    pdal::Dimension const& cdimX = cs.getDimension("X");
    pdal::Dimension const& cdimY = cs.getDimension("Y");
    pdal::Dimension const& cdimZ = cs.getDimension("Z");
    pdal::Dimension const& cdimIntensity = cs.getDimension("Intensity");
    pdal::Dimension const& cdimRed = cs.getDimension("Red");
    pdal::Dimension const& cdimGreen = cs.getDimension("Green");
    pdal::Dimension const& cdimBlue = cs.getDimension("Blue");
    // 
    // int X[] = { 49405730, 49413382, 49402110, 494192890, 49418622, 49403411 };
    // int Y[] = { 487743335, 487743982, 487743983, 487744219, 487744254, 487745019 };
    // int Z[] = { 13063, 13044, 13046, 13050, 13049, 13066 };
    // int I[] = { 134, 75, 153, 93, 67, 167 };
    // int R[] = { 142, 152, 146, 104, 113, 163 };
    // int G[] = { 102, 108, 104, 96, 97, 118 };
    // int B[] = { 137, 134, 140, 120, 123, 150 };
    //     
    for (unsigned i = 0; i < candidate.getNumPoints(); ++i)
    {
        boost::int32_t sx = source.getField<boost::int32_t>(sdimX, i);
        boost::int32_t sy = source.getField<boost::int32_t>(sdimY, i);
        boost::int32_t sz = source.getField<boost::int32_t>(sdimZ, i);
        boost::uint16_t sintensity = source.getField<boost::uint16_t>(sdimIntensity, i);
        boost::uint16_t sred = source.getField<boost::uint16_t>(sdimRed, i);
        boost::uint16_t sgreen = source.getField<boost::uint16_t>(sdimGreen, i);
        boost::uint16_t sblue = source.getField<boost::uint16_t>(sdimBlue, i);

        boost::int32_t cx = candidate.getField<boost::int32_t>(cdimX, i);
        boost::int32_t cy = candidate.getField<boost::int32_t>(cdimY, i);
        boost::int32_t cz = candidate.getField<boost::int32_t>(cdimZ, i);
        boost::uint16_t cintensity = candidate.getField<boost::uint16_t>(cdimIntensity, i);
        boost::uint16_t cred = candidate.getField<boost::uint16_t>(cdimRed, i);
        boost::uint16_t cgreen = candidate.getField<boost::uint16_t>(cdimGreen, i);
        boost::uint16_t cblue = candidate.getField<boost::uint16_t>(cdimBlue, i);


        BOOST_CHECK_EQUAL(sx, cx);
        BOOST_CHECK_EQUAL(sy, cy);
        BOOST_CHECK_EQUAL(sz, cz);
        BOOST_CHECK_EQUAL(sintensity, cintensity);
        BOOST_CHECK_EQUAL(sred, cred);
        BOOST_CHECK_EQUAL(sgreen, cgreen);
        BOOST_CHECK_EQUAL(sblue, cblue);
    }
    delete iter_c;
    delete iter_s;
    
}

BOOST_AUTO_TEST_SUITE_END()
