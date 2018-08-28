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

#include <array>

#include <pdal/Filter.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <io/BpfReader.hpp>
#include <io/BpfWriter.hpp>
#include <io/BufferReader.hpp>
#include <pdal/pdal_features.hpp>

#include "Support.hpp"

using namespace pdal;

namespace
{

template<typename LeftIter, typename RightIter>
::testing::AssertionResult CheckEqualCollections(
    LeftIter left_begin, LeftIter left_end, RightIter right_begin)
{
    bool equal(true);
    std::string message;
    size_t index(0);
    while (left_begin != left_end)
    {
        if (*left_begin++ != *right_begin++)
        {
            equal = false;
            message += "\n\tMismatch at index " + std::to_string(index);
        }
        ++index;
    }
    if (message.size())
        message += "\n\t";
    return equal ? ::testing::AssertionSuccess() :
        ::testing::AssertionFailure() << message;
}



void test_file_type_view(const std::string& filename)
{
    PointTable table;

    struct PtData
    {
        float x;
        float y;
        float z;
    };

    Options ops;

    ops.add("filename", filename);
    ops.add("count", 506);
    std::shared_ptr<BpfReader> reader(new BpfReader);
    reader->setOptions(ops);

    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);

    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 506u);

    PtData pts2[3] = { {494057.312f, 4877433.5f, 130.630005f},
                       {494133.812f, 4877440.0f, 130.440002f},
                       {494021.094f, 4877440.0f, 130.460007f} };

    for (int i = 0; i < 3; ++i)
    {
        float x = view->getFieldAs<float>(Dimension::Id::X, i);
        float y = view->getFieldAs<float>(Dimension::Id::Y, i);
        float z = view->getFieldAs<float>(Dimension::Id::Z, i);

        EXPECT_FLOAT_EQ(x, pts2[i].x);
        EXPECT_FLOAT_EQ(y, pts2[i].y);
        EXPECT_FLOAT_EQ(z, pts2[i].z);
    }

    PtData pts[3] = { {494915.25f, 4878096.5f, 128.220001f},
                      {494917.062f, 4878124.5f, 128.539993f},
                      {494920.781f, 4877914.5f, 127.42999f} };

    for (int i = 0; i < 3; ++i)
    {
        float x = view->getFieldAs<float>(Dimension::Id::X, 503 + i);
        float y = view->getFieldAs<float>(Dimension::Id::Y, 503 + i);
        float z = view->getFieldAs<float>(Dimension::Id::Z, 503 + i);

        EXPECT_FLOAT_EQ(x, pts[i].x);
        EXPECT_FLOAT_EQ(y, pts[i].y);
        EXPECT_FLOAT_EQ(z, pts[i].z);
    }
}

void test_file_type_stream(const std::string& filename)
{
    class Checker : public Filter, public Streamable
    {
    public:
        Checker() : m_cnt(0)
        {}

        struct PtData
        {
            float x;
            float y;
            float z;
        };

        std::string getName() const
        { return "checker"; }

        bool processOne(PointRef& p)
        {
            PtData pts0[3] = { {494057.312f, 4877433.5f, 130.630005f},
                {494133.812f, 4877440.0f, 130.440002f},
                {494021.094f, 4877440.0f, 130.460007f} };

            PtData pts503[3] = { {494915.25f, 4878096.5f, 128.220001f},
                {494917.062f, 4878124.5f, 128.539993f},
                {494920.781f, 4877914.5f, 127.42999f} };

            PtData d;

            if (m_cnt < 3)
               d = pts0[0 + m_cnt];
            else if (m_cnt >= 503 && m_cnt < 506)
               d = pts503[m_cnt - 503];
            else
            {
                m_cnt++;
                return true;
            }

            float x = p.getFieldAs<float>(Dimension::Id::X);
            float y = p.getFieldAs<float>(Dimension::Id::Y);
            float z = p.getFieldAs<float>(Dimension::Id::Z);

            EXPECT_FLOAT_EQ(x, d.x);
            EXPECT_FLOAT_EQ(y, d.y);
            EXPECT_FLOAT_EQ(z, d.z);
            EXPECT_TRUE(m_cnt < 506) << "Count exceeded amount requested "
                "in 'count' option.";

            m_cnt++;
            return true;
        }

    private:
        size_t m_cnt;
    };

    FixedPointTable table(50);

    Options ops;

    ops.add("filename", filename);
    ops.add("count", 506);
    BpfReader reader;
    reader.setOptions(ops);

    Checker c;
    c.setInput(reader);

    c.prepare(table);
    c.execute(table);
}


void test_file_type(const std::string& filename)
{
    test_file_type_view(filename);
    test_file_type_stream(filename);
}


void test_roundtrip(Options& writerOps)
{
    std::string infile(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
    std::string outfile(Support::temppath("tmp.bpf"));

    PointTable table;

    Options readerOps;

    readerOps.add("filename", infile);
    BpfReader reader;
    reader.setOptions(readerOps);

    writerOps.add("filename", outfile);
    BpfWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    FileUtils::deleteFile(outfile);
    writer.prepare(table);
    writer.execute(table);

    test_file_type(outfile);
}

} //namespace

