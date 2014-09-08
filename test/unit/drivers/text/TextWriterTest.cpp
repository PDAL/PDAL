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
#include <pdal/drivers/text/Writer.hpp>

#include <pdal/Utils.hpp>

#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(TextWriterTest)

#ifndef USE_PDAL_PLUGIN_TEXT
BOOST_AUTO_TEST_CASE(TextWriterTest_test_1)
{

    // Only needed for testing using the text driver as a plugin, which
    // is not going to be enabled by default.

    // std::ostringstream oss;
    // std::string p = Support::binpath();
    // oss << "PDAL_DRIVER_PATH="<<p;
    //
    // // We need to keep a copy of this string alive for the duration.
    // std::string path = oss.str();
    // Utils::putenv(path.c_str() );
    pdal::Option option("filename", Support::datapath("pipeline/pipeline_csv.xml"));

    pdal::PipelineManager manager;

    pdal::PipelineReader reader(manager, false, 0);
    reader.readPipeline(option.getValue<std::string>());

    const boost::uint64_t np = manager.execute();


    BOOST_CHECK_EQUAL(np, 106u);

    std::string output_file(Support::temppath("autzen/autzen-point-format-3.txt"));

    bool were_equal = Support::compare_text_files(output_file, Support::datapath("autzen/autzen-point-format-3.txt"));
    BOOST_CHECK(were_equal);
    if (were_equal)
        pdal::FileUtils::deleteFile(output_file);
    else
        std::cout << "comparison of " << Support::datapath("autzen/autzen-point-format-3.txt") << " and " << output_file << " failed.";
}


BOOST_AUTO_TEST_CASE(TextWriterTest_geojson)
{

    pdal::Options options;
    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");
    // options.add(debug);
    // options.add(verbose);
    pdal::Option spatialreference("spatialreference","EPSG:2993", "Output SRS to reproject to");
    pdal::Option filename("filename", Support::datapath("las/1.2-with-color.las"), "");
    pdal::Option ignore("ignore_default", true, "");
    pdal::Option output_type("format", "geojson", "text writer output type");

    pdal::Option keep("keep", "", "");
    pdal::Options keeps;
    pdal::Option x("dimension", "X", "x dim");
    pdal::Option y("dimension", "Y", "y dim");
    pdal::Option z("dimension", "Z", "z dim");
    pdal::Option intensity("dimension", "Intensity", "intensity dim");
    pdal::Option ReturnNumber("dimension", "ReturnNumber", "return dim");

    keeps.add(x); keeps.add(y); keeps.add(z); keeps.add(ReturnNumber); keeps.add(intensity);
    keep.setOptions(keeps);
    options.add(keep);
    options.add(spatialreference);
    options.add(filename);
    options.add(ignore);


    pdal::drivers::las::Reader reader(options);
    pdal::filters::Selector filter(options);
    filter.setInput(&reader);

    std::string output("TextWriterTest-geojson.json");
    pdal::Options writer_options;
    pdal::Option out_filename("filename", Support::temppath(output));
    writer_options.add(out_filename);
    writer_options.add(output_type);

    pdal::drivers::text::Writer writer(writer_options);
    writer.setInput(&filter);

    BOOST_CHECK_EQUAL(writer.getDescription(), "Text Writer");
    writer.prepare();

    writer.write(20);

    bool filesSame = Support::compare_files(Support::temppath(output), Support::datapath(output));
    BOOST_CHECK_EQUAL(filesSame, true);

    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath(output));
    }
}
#endif

BOOST_AUTO_TEST_SUITE_END()
