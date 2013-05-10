/******************************************************************************
* Copyright (c) 2012, Howard Butler (hobu@hobu.net)
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


#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/filters/Selector.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/prc/Writer.hpp>

#include <pdal/Utils.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(PRCWriterTest)

BOOST_AUTO_TEST_CASE(PRCWriterTest_foo)
{
    
    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");
    pdal::Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    
    options.add(filename);

    pdal::drivers::las::Reader reader(options);
    pdal::filters::Selector filter(reader, options);
    
    std::string output("PRCWriterTest-foo.prc");
    pdal::Options writer_options;
    pdal::Option out_filename("filename", Support::temppath(output));
    writer_options.add(out_filename);
    
    pdal::drivers::prc::Writer writer(filter, writer_options);

    BOOST_CHECK_EQUAL(writer.getDescription(), "PRC Writer");
    writer.initialize();

    writer.write(20);

    bool filesSame = Support::compare_files(Support::temppath(output), Support::datapath(output));
    BOOST_CHECK_EQUAL(filesSame, true);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath(output));
    }    

    return;
}

BOOST_AUTO_TEST_SUITE_END()
