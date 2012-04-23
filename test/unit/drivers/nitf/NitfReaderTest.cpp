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
#include <boost/cstdint.hpp>

#include <boost/uuid/uuid_io.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/nitf/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include "Support.hpp"

#include <iostream>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

using namespace pdal;

BOOST_AUTO_TEST_SUITE(NitfReaderTest)

BOOST_AUTO_TEST_CASE(test_one)
{
    //
    // read NITF
    //
    pdal::Option nitf_opt("filename", Support::datapath("nitf/autzen-utm10.ntf"));
    pdal::Options nitf_opts;
    nitf_opts.add(nitf_opt);

    pdal::drivers::nitf::Reader nitf_reader(nitf_opts);
    nitf_reader.initialize();

    BOOST_CHECK_EQUAL(nitf_reader.getDescription(), "NITF Reader");

    // check metadata
    {
        pdal::Metadata metadata = nitf_reader.getMetadata();
        /////////////////////////////////////////////////BOOST_CHECK_EQUAL(metadatums.size(), 80u);


        pdal::metadata::Entry const& m = metadata.getEntry("FH.FDT");

        BOOST_CHECK_EQUAL(m.getName(), "FH.FDT");

        const std::string s(m.getValue<std::string>());
        BOOST_CHECK_EQUAL(s, "20120323002946");
    }

    const Schema& nitf_schema = nitf_reader.getSchema();

    PointBuffer nitf_data(nitf_schema, 750);

    StageSequentialIterator* nitf_iter = nitf_reader.createSequentialIterator(nitf_data);
    const boost::uint32_t nitf_numRead = nitf_iter->read(nitf_data);

    //
    // read LAS
    //
    pdal::Option las_opt("filename", Support::datapath("nitf/autzen-utm10.las"));
    pdal::Options las_opts;
    las_opts.add(las_opt);

    pdal::drivers::las::Reader las_reader(las_opts);
    las_reader.initialize();
    const Schema& las_schema = las_reader.getSchema();

    PointBuffer las_data(las_schema, 750);

    StageSequentialIterator* las_iter = las_reader.createSequentialIterator(las_data);
    const boost::uint32_t las_numRead = las_iter->read(las_data);

    //
    // compare the two buffers
    //
    BOOST_CHECK_EQUAL(las_numRead, nitf_numRead);

    Dimension const& nitf_dimX = nitf_schema.getDimension("X");
    Dimension const& nitf_dimY = nitf_schema.getDimension("Y");
    Dimension const& nitf_dimZ = nitf_schema.getDimension("Z");

    Dimension const& las_dimX = las_schema.getDimension("X");
    Dimension const& las_dimY = las_schema.getDimension("Y");
    Dimension const& las_dimZ = las_schema.getDimension("Z");

    for (boost::uint32_t i=0; i<las_numRead; i++)
    {
        const double nitf_x = nitf_data.getField<double>(nitf_dimX, i);
        const double nitf_y = nitf_data.getField<double>(nitf_dimY, i);
        const double nitf_z = nitf_data.getField<double>(nitf_dimZ, i);

        const double las_x = las_data.getField<double>(las_dimX, i);
        const double las_y = las_data.getField<double>(las_dimY, i);
        const double las_z = las_data.getField<double>(las_dimZ, i);

        BOOST_CHECK_CLOSE(nitf_x, las_x, 0.00001);
        BOOST_CHECK_CLOSE(nitf_y, las_y, 0.00001);
        BOOST_CHECK_CLOSE(nitf_z, las_z, 0.00001);
    }

    delete nitf_iter;
    delete las_iter;

    return;
}


BOOST_AUTO_TEST_SUITE_END()
