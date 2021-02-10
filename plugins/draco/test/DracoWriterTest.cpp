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
        //read las file
        Options o1;
        o1.add("filename", name1);

        LasReader r1;
        r1.setOptions(o1);

        PointTable t1;
        r1.prepare(t1);
        PointViewSet s1 = r1.execute(t1);

        PointViewPtr v1 = *s1.begin();
        DimTypeList d1 = v1->dimTypes();
        point_count_t size1 = v1->size();
        std::vector<char> buf1(size1);

        //read draco file
        Options o2;
        o2.add("filename", name2);

        DracoReader r2;
        r2.setOptions(o2);

        PointTable t2;
        r2.prepare(t2);
        PointViewSet s2 = r2.execute(t2);

        PointViewPtr v2 = *s2.begin();
        DimTypeList d2 = v2->dimTypes();
        point_count_t size2 = v2->size();
        std::vector<char> buf2(size2);


        //compare the two point clouds point count
        EXPECT_EQ(size1, size2);

        //Iterate through all the dimensions in the las file to make sure they
        //all exist in the draco file. Because draco adds texture dims to their
        //files we can't do a straight comparison.
        bool flag = true;
        for (auto& dim1: d1) {
            for (unsigned long i = 0; i < d2.size(); ++i) {
                //dimension is found, break
                if (dim1.m_id == d2[i].m_id && dim1.m_type == d2[i].m_type) break;
                //if not, and we're at the end of the vector, set flag to false
                if (i == (d2.size() - 1)) flag = false;
            }
            if (flag == false) break;
        }
        EXPECT_TRUE(flag);

        //go through all position points and make sure XYZ are the same
        for (PointId i = 0; i < (std::min)(size1, size2); i += 1) {
            std::vector<double> data1(3, 0.0);
            data1[0] = v1->getFieldAs<double>(Dimension::Id::X, i);
            data1[1] = v1->getFieldAs<double>(Dimension::Id::Y, i);
            data1[2] = v1->getFieldAs<double>(Dimension::Id::Z, i);

            std::vector<double> data2(3, 0.0);
            data2[0] = v2->getFieldAs<double>(Dimension::Id::X, i);
            data2[1] = v2->getFieldAs<double>(Dimension::Id::Y, i);
            data2[2] = v2->getFieldAs<double>(Dimension::Id::Z, i);

            EXPECT_EQ(data1, data2);
        }
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
        std::string outFile = Support::temppath("draco_test_dims.drc");
        FileUtils::deleteFile(outFile);

        //setup reader
        Options readerOptions;
        readerOptions.add("filename", inFile);
        readerOptions.add("count", 1065);
        LasReader reader;
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

    void testZeroFill() {
        std::string inFile = Support::temppath("draco_test_dims.drc");

        //setup reader
        Options readerOptions;
        readerOptions.add("filename", inFile);
        // readerOptions.add("count", 1065);
        DracoReader reader;
        reader.setOptions(readerOptions);

        PointTable table;
        reader.prepare(table);
        PointViewSet viewSet = reader.execute(table);
        PointViewPtr view = *viewSet.begin();
        point_count_t count = view->size();

        for (PointId i = 0; i < count; i += 1) {
            float y = view->getFieldAs<float>(Dimension::Id::Y, i);
            float z = view->getFieldAs<float>(Dimension::Id::Z, i);
            EXPECT_EQ(y, 0);
            EXPECT_EQ(z, 0);
        }

    }

    TEST_F(DracoWriterTest, dimensions)
    {
        NL::json dims;
        //test position
        dims = { {"X", "float"}, {"Y", "float"}, {"Z", "double"} };
        testDimensions(dims, false);
        dims = { {"X", "float"}, {"Y", "float"}, {"Z", "float"} };
        testDimensions(dims, true);
        dims = { {"X", "double"} };
        testDimensions(dims, true);
        testZeroFill();


        //test RGB
        dims = { {"Red", "double"}, {"Green", "double"}, {"Blue", "double"} };
        testDimensions(dims, true);
        dims = { {"Red", "double"}, {"Green", "double"}, {"Blue", "uint16"} };
        testDimensions(dims, false);


        //test file doesn't currently use these types
        //test normals
        // dims = { {"NormalX", "double"}, {"NormalY", "double"}, {"NormalZ", "float"} };
        // testDimensions(dims, false);
        // dims = { {"NormalX", "double"} };
        // testDimensions(dims, false);
        // dims = { {"NormalX", "double"}, {"NormalY", "double"}, {"NormalZ", "double"} };
        // testDimensions(dims, true);

        // // test textures
        // dims = { {"TextureU", "double"}, {"TextureV", "double"}, {"TextureW", "float"} };
        // testDimensions(dims, false);
        // dims = { {"TextureU", "double"} };
        // testDimensions(dims, false);
        // dims = { {"TextureU", "double"}, {"TextureV", "double"}, {"TextureW", "double"} };
        // testDimensions(dims, true);


    }


}
