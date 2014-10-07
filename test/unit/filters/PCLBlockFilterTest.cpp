/******************************************************************************
* Copyright (c) 2013, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/PCLBlock.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(PCLBlockFilterTest)

#ifdef PDAL_HAVE_PCL

using namespace pdal;


BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_example_passthrough_xml)
{
    PipelineManager pipeline;
    PipelineReader pipelineReader(pipeline);
    pipelineReader.readPipeline(Support::datapath("filters/pcl/passthrough.xml"));

    pipeline.execute();
    PointContext ctx = pipeline.context();

    PointBufferSet pbSet = pipeline.buffers();
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 81);
}


BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_example_passthrough_json)
{
    Options options;

    Option filename("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    Option debug("debug", true, "");
    Option verbose("verbose", 9, "");

    options.add(filename);
    options.add(debug);
    options.add(verbose);

    drivers::las::Reader reader(options);

    Option fname("filename", Support::datapath("filters/pcl/passthrough.json"));
    Options filter_options;
    filter_options.add(fname);

    filters::PCLBlock pcl_block(filter_options);
    pcl_block.setInput(&reader);

    PointContext ctx;
    pcl_block.prepare(ctx);
    PointBufferSet pbSet = pcl_block.execute(ctx);

    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();
    BOOST_CHECK_EQUAL(buf->size(), 81);
}


BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_example_outlier_json)
{
    Options options;
    Option filename("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    options.add(filename);

    drivers::las::Reader reader(options);

    Option fname("filename",
                 Support::datapath("filters/pcl/passthrough_outlier_removal.json"));
    Options filter_options;
    filter_options.add(fname);
    filters::PCLBlock pcl_block(filter_options);
    pcl_block.setInput(&reader);

    PointContext ctx;
    pcl_block.prepare(ctx);
    PointBufferSet pbSet = pcl_block.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    BOOST_CHECK_EQUAL(buf->size(), 50);
}


BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_example_pmf_json)
{
    Options options;
    Option filename("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    options.add(filename);

    drivers::las::Reader reader(options);

    Option fname("filename",
        Support::datapath("filters/pcl/progressive_morphological_filter.json"));
    Options filter_options;
    filter_options.add(fname);
    filters::PCLBlock pcl_block(filter_options);
    pcl_block.setInput(&reader);

    PointContext ctx;
    pcl_block.prepare(ctx);
    PointBufferSet pbSet = pcl_block.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    BOOST_CHECK_EQUAL(buf->size(), 93);
}


BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_pmf2_json)
{
    Options options;
    Option filename("filename", Support::datapath("autzen/autzen-point-format-3.las"));
    options.add(filename);

    drivers::las::Reader reader(options);

    Option fname("filename",
        Support::datapath("filters/pcl/progressive_morphological_filter2.json"));
    Options filter_options;
    filter_options.add(fname);
    filters::PCLBlock pcl_block(filter_options);
    pcl_block.setInput(&reader);

    PointContext ctx;
    pcl_block.prepare(ctx);
    PointBufferSet pbSet = pcl_block.execute(ctx);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr buf = *pbSet.begin();

    BOOST_CHECK_EQUAL(buf->size(), 94);
}


#endif // PDAL_HAVE_PCL

BOOST_AUTO_TEST_SUITE_END()
