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
#include <boost/cstdint.hpp>

#include <boost/uuid/uuid_io.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/buffer/Reader.hpp>
#include <pdal/drivers/text/Writer.hpp>

#include <iostream>

#include "Support.hpp"

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wfloat-equal"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif

using namespace pdal;

PointBuffer *makeBuffer()
{
    // create a input
    std::vector<pdal::Dimension> dimensions;
    dimensions.reserve(3);

    pdal::Dimension dimX("X", pdal::dimension::Float, 4,
                         "X coordinate as a float.");
    dimX.setUUID("2ee118d1-119e-4906-99c3-42934203f872");
    dimX.setNamespace("X");
    dimensions.push_back(dimX);

    pdal::Dimension dimY("Y", pdal::dimension::Float, 4,
                         "Y coordinate as a float.");
    dimY.setUUID("87707eee-2f30-4979-9987-8ef747e30275");
    dimY.setNamespace("Y");
    dimensions.push_back(dimY);

    pdal::Dimension dimZ("Z", pdal::dimension::Float, 4,
                         "Z coordinate as a float.");
    dimZ.setUUID("e74b5e41-95e6-4cf2-86ad-e3f5a996da5d");
    dimZ.setNamespace("Z");
    dimensions.push_back(dimZ);

    const pdal::Schema schema(dimensions);

    boost::uint32_t numPoints = 1000;
    pdal::PointBuffer *input(new pdal::PointBuffer(schema, numPoints));

    input->setNumPoints(0);

    pdal::Bounds<double> bounds;
    bool bFirstPoint(true);

    for (boost::uint32_t i=0; i<numPoints; i++)
    {
        input->setField<float>(schema.getDimension("X"), i, 1.0);
        input->setField<float>(schema.getDimension("Y"), i, 2.0);
        input->setField<float>(schema.getDimension("Z"), i, 3.0);

        if (bFirstPoint)
        {
            bounds = pdal::Bounds<double>(1.0, 2.0, 3.0, 1.0, 2.0, 3.0);
            bFirstPoint = false;
        }

        bounds.grow(pdal::Vector<double>(1.0, 2.0, 3.0));
        input->setNumPoints(i+1);
    }

    input->setSpatialBounds(bounds);

    return input;
}

BOOST_AUTO_TEST_SUITE(BufferReaderTest)

BOOST_AUTO_TEST_CASE(test_sequential_iter)
{
    PointBuffer *input = makeBuffer();

    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");

    pdal::Options reader_options;
    // reader_options.add(debug);
    // reader_options.add(verbose);

    pdal::drivers::buffer::Reader reader(reader_options, *input);
    reader.initialize();

    BOOST_CHECK_EQUAL(reader.getDescription(), "Buffer Reader");

    const Schema& schema = reader.getSchema();

    PointBuffer data(schema, 750);

    StageSequentialIterator* iter = reader.createSequentialIterator(data);
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 750);

    Schema const& buffer_schema = data.getSchema();
    Dimension const& dimX = buffer_schema.getDimension("X");
    Dimension const& dimY = buffer_schema.getDimension("Y");
    Dimension const& dimZ = buffer_schema.getDimension("Z");

    for (boost::uint32_t i=0; i<numRead; i++)
    {
        float x = data.getField<float>(dimX, i);
        float y = data.getField<float>(dimY, i);
        float z = data.getField<float>(dimZ, i);

        BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
        BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
        BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
    }

    delete iter;
    delete input;
}

BOOST_AUTO_TEST_CASE(test_random_iter)
{
    PointBuffer *input = makeBuffer();

    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");

    pdal::Options reader_options;
    // reader_options.add(debug);
    // reader_options.add(verbose);

    pdal::drivers::buffer::Reader reader(reader_options, *input);
    reader.initialize();

    BOOST_CHECK_EQUAL(reader.getDescription(), "Buffer Reader");

    const Schema& schema = reader.getSchema();

    PointBuffer data(schema, 10);

    StageRandomIterator* iter = reader.createRandomIterator(data);

    boost::uint32_t numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10u);

    Schema const& buffer_schema = data.getSchema();
    Dimension const& dimX = buffer_schema.getDimension("X");
    Dimension const& dimY = buffer_schema.getDimension("Y");
    Dimension const& dimZ = buffer_schema.getDimension("Z");


    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            float x = data.getField<float>(dimX, i);
            float y = data.getField<float>(dimY, i);
            float z = data.getField<float>(dimZ, i);

            BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
            BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
            BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        }
    }

    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10u);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            float x = data.getField<float>(dimX, i);
            float y = data.getField<float>(dimY, i);
            float z = data.getField<float>(dimZ, i);

            BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
            BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
            BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        }
    }

    boost::uint64_t newPos = iter->seek(99);
    BOOST_CHECK_EQUAL(newPos, 99u);
    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10u);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            float x = data.getField<float>(dimX, i);
            float y = data.getField<float>(dimY, i);
            float z = data.getField<float>(dimZ, i);

            BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
            BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
            BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        }
    }

    newPos = iter->seek(7);
    BOOST_CHECK_EQUAL(newPos, 7u);
    numRead = iter->read(data);
    BOOST_CHECK_EQUAL(numRead, 10u);

    {
        for (boost::uint32_t i=0; i<numRead; i++)
        {
            float x = data.getField<float>(dimX, i);
            float y = data.getField<float>(dimY, i);
            float z = data.getField<float>(dimZ, i);

            BOOST_CHECK_CLOSE(x, 1.0, 0.00001);
            BOOST_CHECK_CLOSE(y, 2.0, 0.00001);
            BOOST_CHECK_CLOSE(z, 3.0, 0.00001);
        }
    }

    delete iter;
    delete input;
}


BOOST_AUTO_TEST_CASE(test_iterator_write)
{
    PointBuffer *input = makeBuffer();

    pdal::Option debug("debug", true, "");
    pdal::Option verbose("verbose", 5, "");

    pdal::Options reader_options;
    // reader_options.add(debug);
    // reader_options.add(verbose);

    pdal::drivers::buffer::Reader reader(reader_options, *input);
    //reader.initialize();

    std::string output("BufferReaderTest-geojson.json");
    pdal::Option out_filename("filename", Support::temppath(output));
    pdal::Option output_type("format", "geojson", "");

    pdal::Options writer_opts;
    // writer_opts.add(debug);
    // writer_opts.add(verbose);
    writer_opts.add(out_filename);
    writer_opts.add(output_type);

    pdal::drivers::text::Writer writer(reader, writer_opts);
    writer.initialize();
    boost::uint64_t numWritten = writer.write(100);

    BOOST_CHECK_EQUAL(numWritten, 100u);
    FileUtils::deleteFile(out_filename.getValue<std::string>());
    
    delete input;
}


BOOST_AUTO_TEST_SUITE_END()
