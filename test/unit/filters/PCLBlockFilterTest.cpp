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

#include <pdal/SpatialReference.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/PCLBlock.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(PCLBlockFilterTest)

#ifdef PDAL_HAVE_PCL

BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_passthrough)
{
        pdal::Options options;
        
        pdal::Option filename("filename", Support::datapath("autzen-point-format-3.las"));
        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");

        options.add(filename);
        options.add(debug);
        options.add(verbose);

        pdal::drivers::las::Reader reader(options);

        pdal::Option fname("filename", Support::datapath("pipeline/passthrough.json"));

        pdal::Options filter_options;

        filter_options.add(fname);

        pdal::filters::PCLBlock pcl_block(reader, filter_options);
        
        pcl_block.initialize();

        const pdal::Schema& schema = reader.getSchema();
        pdal::PointBuffer data(schema, reader.getNumPoints());
        
        pdal::StageSequentialIterator* iter = pcl_block.createSequentialIterator(data);

        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 81u);

        pdal::filters::iterators::sequential::PCLBlock* b = static_cast<pdal::filters::iterators::sequential::PCLBlock*>(iter);

        delete iter;

    return;
}

BOOST_AUTO_TEST_CASE(PCLBlockFilterTest_outlier_removal)
{
        pdal::Options options;
        
        pdal::Option filename("filename", Support::datapath("autzen-point-format-3.las"));
        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 9, "");

        options.add(filename);
        options.add(debug);
        options.add(verbose);

        pdal::drivers::las::Reader reader(options);

        pdal::Option fname("filename", Support::datapath("pipeline/outlier_removal.json"));

        pdal::Options filter_options;

        filter_options.add(fname);

        pdal::filters::PCLBlock pcl_block(reader, filter_options);
        
        pcl_block.initialize();

        const pdal::Schema& schema = reader.getSchema();
        pdal::PointBuffer data(schema, reader.getNumPoints());
        
        pdal::StageSequentialIterator* iter = pcl_block.createSequentialIterator(data);

        boost::uint32_t numRead = iter->read(data);
        BOOST_CHECK_EQUAL(numRead, 100u);

        pdal::filters::iterators::sequential::PCLBlock* b = static_cast<pdal::filters::iterators::sequential::PCLBlock*>(iter);

        delete iter;

    return;
}

#endif

BOOST_AUTO_TEST_SUITE_END()

