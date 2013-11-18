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

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/nitf/Reader.hpp>
#include <pdal/drivers/nitf/Writer.hpp>
#include "Support.hpp"

using namespace pdal;

BOOST_AUTO_TEST_SUITE(NitfWriterTest)

#if 0
static void compare_contents(const std::string& las_file, const std::string& ntf_file)
{
    //
    // read the LAS file
    //
    pdal::Option las_opt("filename", las_file);
    pdal::Options las_opts;
    las_opts.add(las_opt);

    pdal::drivers::las::Reader las_reader(las_opts);
    las_reader.initialize();
    const Schema& las_schema = las_reader.getSchema();
    PointBuffer las_data(las_schema, 750);
    StageSequentialIterator* las_iter = las_reader.createSequentialIterator(las_data);
    const boost::uint32_t las_numRead = las_iter->read(las_data);

    //
    // read the NITF file
    //
    pdal::Option ntf_opt("filename", ntf_file);
    pdal::Options ntf_opts;
    ntf_opts.add(ntf_opt);

    pdal::drivers::nitf::Reader ntf_reader(ntf_opts);
    ntf_reader.initialize();
    const Schema& ntf_schema = ntf_reader.getSchema();
    PointBuffer ntf_data(ntf_schema, 750);
    StageSequentialIterator* ntf_iter = ntf_reader.createSequentialIterator(ntf_data);
    const boost::uint32_t ntf_numRead = ntf_iter->read(ntf_data);

    //
    // compare the two buffers
    //
    BOOST_CHECK_EQUAL(las_numRead, ntf_numRead);

    Dimension const& ntf_dimX = ntf_schema.getDimension("X");
    Dimension const& ntf_dimY = ntf_schema.getDimension("Y");
    Dimension const& ntf_dimZ = ntf_schema.getDimension("Z");

    Dimension const& las_dimX = las_schema.getDimension("X");
    Dimension const& las_dimY = las_schema.getDimension("Y");
    Dimension const& las_dimZ = las_schema.getDimension("Z");

    for (boost::uint32_t i=0; i<las_numRead; i++)
    {
        const double ntf_x = ntf_data.getField<double>(ntf_dimX, i);
        const double ntf_y = ntf_data.getField<double>(ntf_dimY, i);
        const double ntf_z = ntf_data.getField<double>(ntf_dimZ, i);

        const double las_x = las_data.getField<double>(las_dimX, i);
        const double las_y = las_data.getField<double>(las_dimY, i);
        const double las_z = las_data.getField<double>(las_dimZ, i);

        BOOST_CHECK_CLOSE(ntf_x, las_x, 0.00001);
        BOOST_CHECK_CLOSE(ntf_y, las_y, 0.00001);
        BOOST_CHECK_CLOSE(ntf_z, las_z, 0.00001);
    }

    delete ntf_iter;
    delete las_iter;

    return;
}
#endif

BOOST_AUTO_TEST_CASE(test1)
{

#ifdef PDAL_HAVE_NITRO
    const std::string las_input(Support::datapath("1.2-with-color.las"));
    const std::string nitf_output(Support::temppath("temp_nitf.ntf"));
    const std::string reference_output(Support::datapath("nitf/write_test1.ntf"));

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
        
        pdal::drivers::las::Reader reader(reader_opts);

        pdal::drivers::nitf::Writer writer(reader, writer_opts);
        {
            // writer.setCompressed(false);
            // // writer.setDate(0, 0);
            // // writer.setPointFormat(::pdal::drivers::las::PointFormat3);
            // // writer.setSystemIdentifier("");
            // writer.setGeneratingSoftware("PDAL-NITF");
            // writer.setChunkSize(100);
        }
        writer.initialize();

        writer.write(0);
    }
    
    FileUtils::deleteFile(nitf_output);

#endif

    return;

#if 0
    //
    // check the generated NITF
    //
    bool filesSame = Support::compare_files(nitf_output, reference_output);
    BOOST_CHECK(filesSame);

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

    return;
}


BOOST_AUTO_TEST_SUITE_END()
