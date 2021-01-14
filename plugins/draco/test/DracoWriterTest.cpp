/******************************************************************************
* Copyright (c) 2020, Hobu, Inc.
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

#define NOMINMAX

#include <stdio.h>
#include <sys/types.h>

#include <nlohmann/json.hpp>

#include <pdal/pdal_test_main.hpp>
#include <io/FauxReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"
#include "../io/DracoWriter.hpp"
#include "../io/DracoReader.hpp"
#include <io/LasReader.hpp>

namespace pdal
{
    size_t count = 100;
    class DracoWriterTest : public ::testing::Test
    {
    protected:
        virtual void SetUp()
        {
            Options options;
            options.add("mode", "ramp");
            options.add("count", count);
            m_reader.setOptions(options);
            m_reader2.setOptions(options);
        }

        FauxReader m_reader;
        FauxReader m_reader2;

    };

    TEST_F(DracoWriterTest, constructor)
    {
        DracoWriter writer;
    }

    TEST_F(DracoWriterTest, findStage)
    {
        StageFactory factory;
        Stage* stage(factory.createStage("writers.draco"));
        EXPECT_TRUE(stage);
    }

    TEST_F(DracoWriterTest, write)
    {
        Options options;
        options.add("filename", Support::temppath("out.drc"));
        DracoWriter writer;
        writer.setOptions(options);
        writer.setInput(m_reader);

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    void compareFiles(const std::string& name1, const std::string& name2)
    {
        Options o1;
        o1.add("filename", name1);

        LasReader r1;
        r1.setOptions(o1);

        PointTable t1;
        r1.prepare(t1);
        PointViewSet s1 = r1.execute(t1);
        PointViewPtr v1 = *s1.begin();
        DimTypeList d1 = v1->dimTypes();
        size_t size1 = v1->pointSize();
        std::vector<char> buf1(size1);


        Options o2;
        o2.add("filename", name2);

        DracoReader r2;
        r2.setOptions(o2);

        PointTable t2;
        r2.prepare(t2);
        PointViewSet s2 = r2.execute(t2);
        // PointViewPtr v2 = *s2.begin();
        // DimTypeList d2 = v2->dimTypes();
        // size_t size2 = v2->pointSize();
        // std::vector<char> buf2(size2);

        // EXPECT_EQ(v1->size(), v2->size());
        // EXPECT_EQ(d1.size(), d2.size());
        // EXPECT_EQ(size1, size2);

        // for (PointId i = 0; i < (std::min)(size1, size2); i += 1)
        // {
        //     v1->getPackedPoint(d1, i, buf1.data());
        //     v2->getPackedPoint(d2, i, buf2.data());
        //     EXPECT_EQ(memcmp(buf1.data(), buf2.data(), (std::min)(size1, size2)), 0);
        // }
    }

    TEST_F(DracoWriterTest, output)
    {
        std::string inFile = Support::datapath("las/1.2-with-color.las");
        std::string outFile = Support::temppath("draco_test.drc");
        FileUtils::deleteFile(outFile);

        //setup reader
        Options readerOptions;
        readerOptions.add("filename", inFile);
        readerOptions.add("count", 1065);
        LasReader reader;
        reader.setOptions(readerOptions);

        //setup writer
        PointTable table;
        Options writerOptions;
        writerOptions.add("filename", outFile);
        DracoWriter writer;
        writer.setOptions(writerOptions);
        writer.setInput(reader);

        writer.prepare(table);
        writer.execute(table);

        compareFiles(inFile, outFile);
    }

    void testDimensions(NL::json dimensions, bool pass)
    {
        std::string inFile = Support::datapath("las/1.2-with-color.las");
        std::string outFile = Support::temppath("draco_test.drc");
        FileUtils::deleteFile(outFile);

        //setup reader
        Options readerOptions;
        readerOptions.add("filename", inFile);
        readerOptions.add("count", 1065);
        FauxReader reader;
        reader.setOptions(readerOptions);

        Options options;
        options.add("filename", outFile);
        options.add("dimensions", dimensions);

        DracoWriter writer;
        writer.setOptions(options);
        writer.setInput(reader);

        PointTable table;
        writer.prepare(table);

        if (pass) writer.execute(table);
        else EXPECT_THROW(writer.execute(table), pdal_error);
    }

    TEST_F(DracoWriterTest, dimensions)
    {
        NL::json dims;
        //test position
        dims = { {"X", "double"}, {"Y", "double"}, {"Z", "float"} };
        testDimensions(dims, false);
        dims = { {"X", "double"} };
        testDimensions(dims, false);
        dims = { {"X", "double"}, {"Y", "double"}, {"Z", "double"} };
        testDimensions(dims, true);

        //test normals
        dims = { {"NormalX", "double"}, {"NormalY", "double"}, {"NormalZ", "float"} };
        testDimensions(dims, false);
        dims = { {"NormalX", "double"} };
        testDimensions(dims, false);
        dims = { {"NormalX", "double"}, {"NormalY", "double"}, {"NormalZ", "double"} };
        testDimensions(dims, true);

        //test RGB
        dims = { {"Red", "double"}, {"Green", "double"}, {"Blue", "float"} };
        testDimensions(dims, false);
        dims = { {"Red", "double"} };
        testDimensions(dims, false);
        dims = { {"Red", "double"}, {"Green", "double"}, {"Blue", "double"} };
        testDimensions(dims, true);

        //test textures
        // dims = { {"TextureU", "double"}, {"TextureV", "double"}, {"TextureW", "float"} };
        // testDimensions(dims, false);
        // dims = { {"TextureU", "double"} };
        // testDimensions(dims, false);
        // dims = { {"TextureU", "double"}, {"TextureV", "double"}, {"TextureW", "double"} };
        // testDimensions(dims, true);

    }


}
