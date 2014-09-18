/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <boost/uuid/uuid_io.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>

#include <pdal/drivers/nitf/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Chipper.hpp>

#include "Support.hpp"

#include <iostream>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

#include <boost/property_tree/xml_parser.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(NitfReaderTest)

BOOST_AUTO_TEST_CASE(test_one)
{
    Options nitf_opts;
    nitf_opts.add("filename", Support::datapath("nitf/autzen-utm10.ntf"));
    nitf_opts.add("count", 750);

    PointContext ctx;
    drivers::nitf::NitfReader nitf_reader(nitf_opts);
    nitf_reader.prepare(ctx);
    PointBufferSet pbSet = nitf_reader.execute(ctx);
    BOOST_CHECK_EQUAL(nitf_reader.getDescription(), "NITF Reader");
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    // check metadata
//ABELL
/**
    {
        pdal::Metadata metadata = nitf_reader.getMetadata();
        /////////////////////////////////////////////////BOOST_CHECK_EQUAL(metadatums.size(), 80u);
        BOOST_CHECK_EQUAL(metadata.toPTree().get<std::string>("metadata.FH_FDT.value"), "20120323002946");
    }
**/

    //
    // read LAS
    //
    pdal::Options las_opts;
    las_opts.add("count", 750);
    las_opts.add("filename", Support::datapath("nitf/autzen-utm10.las"));

    PointContext ctx2;
    drivers::las::Reader las_reader(las_opts);
    las_reader.prepare(ctx2);
    PointBufferSet pbSet2 = las_reader.execute(ctx2);
    BOOST_CHECK_EQUAL(pbSet2.size(), 1);
    PointBufferPtr buf2 = *pbSet.begin();
    //
    //
    // compare the two buffers
    //
    BOOST_CHECK_EQUAL(buf->size(), buf2->size());

    for (PointId i = 0; i < buf2->size(); i++)
    {
        int32_t nitf_x = buf->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t nitf_y = buf->getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t nitf_z = buf->getFieldAs<int32_t>(Dimension::Id::Z, i);

        int32_t las_x = buf2->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t las_y = buf2->getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t las_z = buf2->getFieldAs<int32_t>(Dimension::Id::Z, i);

        BOOST_CHECK_EQUAL(nitf_x, las_x);
        BOOST_CHECK_EQUAL(nitf_y, las_y);
        BOOST_CHECK_EQUAL(nitf_z, las_z);
    }
}


BOOST_AUTO_TEST_CASE(test_chipper)
{
    pdal::Option option("filename", Support::datapath("nitf/chipper.xml"));
    pdal::Options options(option);

    PointContext ctx;

    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(Support::datapath("nitf/chipper.xml"));
    //ABELL - need faux writer or something.
    /**
    mgr.execute();
    StageSequentialIterator* iter = reader.createSequentialIterator(data);
    const boost::uint32_t num_read = iter->read(data);
    BOOST_CHECK_EQUAL(num_read, 13u);    
    
    boost::uint32_t num_blocks = chipper->GetBlockCount();
    BOOST_CHECK_EQUAL(num_blocks, 8u);
    **/
}

BOOST_AUTO_TEST_SUITE_END()
