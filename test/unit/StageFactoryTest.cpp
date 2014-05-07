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

#include "Support.hpp"

#include <pdal/FileUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/Reader.hpp>
#include <pdal/Filter.hpp>
#include <pdal/Writer.hpp>
#include <pdal/Options.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Mosaic.hpp>

using namespace pdal;

BOOST_AUTO_TEST_SUITE(StageFactoryTest)


BOOST_AUTO_TEST_CASE(StageFactoryTest_test1)
{
    FileUtils::deleteFile("temp.las");

    StageFactory factory;

    Options optsR;
    optsR.add("filename", Support::datapath("1.2-with-color.las"));
    Reader* reader = factory.createReader("drivers.las.reader", optsR);
    BOOST_CHECK(reader->getName() == "drivers.las.reader");

    Options optsF;
    optsF.add("bounds", Bounds<double>(0,0,0,1000000,1000000,1000000));
    Filter* filter = factory.createFilter("filters.crop", *reader, optsF);
    BOOST_CHECK(filter->getName() == "filters.crop");

    Options optsM;
    std::vector<Stage*> stages;
    stages.push_back(filter);
    MultiFilter* multifilter = factory.createMultiFilter("filters.mosaic", stages, optsM);
    BOOST_CHECK(multifilter->getName() == "filters.mosaic");

    Options optsW;
    optsW.add("filename", "temp.las", "file to write to");
    Writer* writer = factory.createWriter("drivers.las.writer", *filter, optsW);
    BOOST_CHECK(writer->getName() == "drivers.las.writer");
    writer->prepare();

    const boost::uint64_t np = writer->write(reader->getNumPoints());
    BOOST_CHECK(np == 1065);

    delete writer;
    delete multifilter;
    delete filter;
    delete reader;

    FileUtils::deleteFile("temp.las");
}


static int s_demoflag = 0;
static Reader* demoReaderCreator(const Options& options)
{
    s_demoflag = options.getOption("flag").getValue<int>();

    // this is where you'd do something like:
    //     return new MyCustomXyzReader(options);

    Options optsR;
    optsR.add("filename", Support::datapath("1.2-with-color.las"), "file to read from");
    Reader* reader = new pdal::drivers::las::Reader(optsR);
    return reader;
}


Filter* demoFilterCreator(Stage& prev, const Options& options)
{
    s_demoflag = options.getOption("flag").getValue<int>();

    Options optsF;
    optsF.add("bounds", Bounds<double>(0,0,0,1,1,1), "crop bounds");
    Filter* filter = new pdal::filters::Crop(optsF);
    filter->setInput(&prev);
    return filter;
}


MultiFilter* demoMultiFilterCreator(const std::vector<Stage*>& prevs, const Options& options)
{
    s_demoflag = options.getOption("flag").getValue<int>();

    const Options optsM;
    MultiFilter* multifilter = new pdal::filters::Mosaic(optsM);
    multifilter->setInput(prevs);
    return multifilter;
}


Writer* demoWriterCreator(Stage& prev, const Options& options)
{
    s_demoflag = options.getOption("flag").getValue<int>();

    Options optsW;
    optsW.add("filename", "temp.las", "file to write to");
    Writer* writer = new pdal::drivers::las::Writer(optsW);
    writer->setInput(&prev);
    return writer;
}


BOOST_AUTO_TEST_CASE(StageFactoryTest_test2)
{
    FileUtils::deleteFile("temp.las");

    StageFactory factory;

    factory.registerReader("demoR", demoReaderCreator);
    factory.registerFilter("demoF", demoFilterCreator);
    factory.registerMultiFilter("demoM", demoMultiFilterCreator);
    factory.registerWriter("demoW", demoWriterCreator);

    s_demoflag = 0;
    Options optsR;
    optsR.add("flag",11,"my flag");
    Stage* reader = factory.createReader("demoR", optsR);
    BOOST_CHECK(reader->getName() == "drivers.las.reader");
    BOOST_CHECK(s_demoflag == 11);

    s_demoflag = 0;
    Options optsF;
    optsF.add("flag",22,"my flag");
    Stage* filter = factory.createFilter("demoF", *reader, optsF);
    BOOST_CHECK(filter->getName() == "filters.crop");
    BOOST_CHECK(s_demoflag == 22);

    s_demoflag = 0;
    Options optsM;
    optsM.add("flag",33,"my flag");
    std::vector<Stage*> stages;
    stages.push_back(reader);
    Stage* multifilter = factory.createMultiFilter("demoM", stages, optsM);
    BOOST_CHECK(multifilter->getName() == "filters.mosaic");
    BOOST_CHECK(s_demoflag == 33);

    s_demoflag = 0;
    Options optsW;
    optsW.add("flag",44,"my flag");
    Writer* writer = factory.createWriter("demoW", *reader, optsW);
    BOOST_CHECK(writer->getName() == "drivers.las.writer");
    writer->prepare();
    BOOST_CHECK(s_demoflag == 44);

    delete writer;
    delete multifilter;
    delete filter;
    delete reader;

    FileUtils::deleteFile("temp.las");
}


BOOST_AUTO_TEST_SUITE_END()
