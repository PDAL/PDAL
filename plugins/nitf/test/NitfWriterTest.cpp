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

#include <pdal/pdal_test_main.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include "Support.hpp"

using namespace pdal;

#if 0
static void compare_contents(const std::string& las_file, const std::string& ntf_file)
{
    //
    // read the LAS file
    //
    Option las_opt("filename", las_file);
    Options las_opts;
    las_opts.add(las_opt);

    LasReader las_reader(las_opts);
    las_reader.prepare();
    const Schema& las_schema = las_reader.getSchema();
    PointViewPtr las_data(las_schema, 750);
    StageSequentialIterator* las_iter = las_reader.createSequentialIterator(las_data);
    const uint32_t las_numRead = las_iter->read(las_data);

    //
    // read the NITF file
    //
    Option ntf_opt("filename", ntf_file);
    Options ntf_opts;
    ntf_opts.add(ntf_opt);

    NitfReader ntf_reader(ntf_opts);
    ntf_reader.prepare();
    const Schema& ntf_schema = ntf_reader.getSchema();
    PointViewPtr ntf_data(ntf_schema, 750);
    StageSequentialIterator* ntf_iter = ntf_reader.createSequentialIterator(ntf_data);
    const uint32_t ntf_numRead = ntf_iter->read(ntf_data);

    //
    // compare the two views
    //
    EXPECT_EQ(las_numRead, ntf_numRead);

    Dimension const& ntf_dimX = ntf_schema.getDimension("X");
    Dimension const& ntf_dimY = ntf_schema.getDimension("Y");
    Dimension const& ntf_dimZ = ntf_schema.getDimension("Z");

    Dimension const& las_dimX = las_schema.getDimension("X");
    Dimension const& las_dimY = las_schema.getDimension("Y");
    Dimension const& las_dimZ = las_schema.getDimension("Z");

    for (uint32_t i=0; i<las_numRead; i++)
    {
        const double ntf_x = ntf_data.getField<double>(ntf_dimX, i);
        const double ntf_y = ntf_data.getField<double>(ntf_dimY, i);
        const double ntf_z = ntf_data.getField<double>(ntf_dimZ, i);

        const double las_x = las_data.getField<double>(las_dimX, i);
        const double las_y = las_data.getField<double>(las_dimY, i);
        const double las_z = las_data.getField<double>(las_dimZ, i);

        EXPECT_FLOAT_EQ(ntf_x, las_x);
        EXPECT_FLOAT_EQ(ntf_y, las_y);
        EXPECT_FLOAT_EQ(ntf_z, las_z);
    }

    delete ntf_iter;
    delete las_iter;
}
#endif

TEST(NitfWriterTest, test1)
{
    StageFactory f;

    const std::string las_input(Support::datapath("las/1.2-with-color.las"));
    const std::string nitf_output(Support::temppath("temp_nitf.ntf"));
    const std::string reference_output(
        Support::datapath("nitf/write_test1.ntf"));

    FileUtils::deleteFile(nitf_output);

    //
    // write the NITF
    //
    {
        Options reader_opts;
        Option reader_opt1("filename", las_input);
        reader_opts.add(reader_opt1);

        Options writer_opts;
        Option writer_opt1("filename", nitf_output);
        Option debug("debug", true);
        Option verbose("verbose", 8);

        Option datetime("IDATIM", "20110516183337");
        writer_opts.add(datetime);

        Option cls("FSCLAS", "S");
        writer_opts.add(cls);

        Option phone("OPHONE", "5159664628");
        writer_opts.add(phone);

        Option name("ONAME", "Howard Butler");
        writer_opts.add(name);

        Option ftitle("FTITLE", "LiDAR from somewhere");
        writer_opts.add(ftitle);

        // writer_opts.add(debug);
        // writer_opts.add(verbose);
        writer_opts.add(writer_opt1);

        std::unique_ptr<Stage> reader(f.createStage("readers.las"));
        EXPECT_TRUE(reader.get());
        reader->setOptions(reader_opts);

        std::unique_ptr<Stage> writer(f.createStage("writers.nitf"));
        EXPECT_TRUE(writer.get());
        writer->setOptions(writer_opts);
        writer->setInput(*reader);
        {
            // writer.setCompressed(false);
            // // writer.setDate(0, 0);
            // // writer.setPointFormat(::drivers::las::PointFormat3);
            // // writer.setSystemIdentifier("");
            // writer.setGeneratingSoftware("PDAL-NITF");
            // writer.setChunkSize(100);
        }
        PointTable table;
        writer->prepare(table);
        writer->execute(table);
    }

    FileUtils::deleteFile(nitf_output);

#if 0
    //
    // check the generated NITF
    //
    bool filesSame = Support::compare_files(nitf_output, reference_output);
    EXPECT_TRUE(filesSame);

    //
    // check the LAS contents against the source image
    //
    compare_contents(las_input, nitf_output);

    //
    // rm temp file
    //
    if (filesSame)
    {
        FileUtils::deleteFile(Support::temppath(nitf_output));
    }
#endif
}
