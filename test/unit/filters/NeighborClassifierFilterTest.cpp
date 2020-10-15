/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include "Support.hpp"

#include <filters/StatsFilter.hpp>

namespace pdal
{

const stats::Summary::EnumMap GetClassifications(Stage &s,
    point_count_t *count = NULL)
{
    StatsFilter stats;
    stats.setInput(s);

    Options statOpts;
    statOpts.add("enumerate", "Classification");
    stats.setOptions(statOpts);

    PointTable table;
    stats.prepare(table);
    PointViewSet viewSet = stats.execute(table);
    const stats::Summary& statsClassification = stats.getStats(Dimension::Id::Classification);
    if (count)
        *count = statsClassification.count();
    return statsClassification.values();
}

TEST(NeighborClassifierFilterTest, singleRange)
{
    Options ro;
    ro.add("filename", Support::datapath("las/sample_c.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);
    point_count_t count = 0;
    stats::Summary::EnumMap OrigClassifications = GetClassifications(r, &count);

    std::vector<unsigned int> kvals = {1, 3};
    for (auto &k : kvals) {

        Options fo;
        fo.add("domain", "Classification[14:14]");
        //fo.add("dimension", "Classification");
        fo.add("k", k);

        Stage& f = *(factory.createStage("filters.neighborclassifier"));
        f.setInput(r);

        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());
        EXPECT_EQ(count, view->size());

        stats::Summary::EnumMap NewClassifications = GetClassifications(f);

        for (auto& p : OrigClassifications)
        {
            if (k == 1)
            {
                EXPECT_TRUE(NewClassifications[p.first] == OrigClassifications[p.first]);
            }
            else
            {
                if (p.first == 14)
                {
                    EXPECT_TRUE(NewClassifications[p.first] == 0);
                }
                else
                    EXPECT_TRUE(NewClassifications[p.first] >= OrigClassifications[p.first]);
            }
        }
    }
}

TEST(NeighborClassifierFilterTest, multipleRange)
{
    Options ro;
    ro.add("filename", Support::datapath("las/sample_c.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);
    point_count_t count = 0;
    stats::Summary::EnumMap OrigClassifications = GetClassifications(r, &count);

    std::vector<unsigned int> kvals = {1, 3};
    for (auto &k : kvals) {

        Options fo;
        fo.add("domain", "Classification[14:14], Classification[11:11]");
        //fo.add("dimension", "Classification");
        fo.add("k", k);

        Stage& f = *(factory.createStage("filters.neighborclassifier"));
        f.setInput(r);
        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());
        EXPECT_EQ(count, view->size());

        stats::Summary::EnumMap NewClassifications = GetClassifications(f);

        for (auto& p : OrigClassifications)
        {
            if (k == 1)
            {
                EXPECT_TRUE(NewClassifications[p.first] == OrigClassifications[p.first]);
            }
            else
            {
                if (p.first == 14 || p.first == 11)
                {
                    EXPECT_TRUE(NewClassifications[p.first] == 0);
                }
                else
                    EXPECT_TRUE(NewClassifications[p.first] >= OrigClassifications[p.first]);
            }
        }
    }
}

TEST(NeighborClassifierFilterTest, candidate)
{
    StageFactory factory;

    Options rClassifications;
    point_count_t count = 0;
    rClassifications.add("filename", Support::datapath("las/sample_c.las"));
    Stage& rC = *(factory.createStage("readers.las"));
    rC.setOptions(rClassifications);
    stats::Summary::EnumMap OrigClassifications =
        GetClassifications(rC, &count);

    Options ro;
    ro.add("filename", Support::datapath("las/sample_nc.las"));

    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    // NeighborClassifier used to be broken because it would change voting point
    // classifications while it was running. This mean it would create different
    // classifications if the point order was different.
    // Randomizing the data should quickly expose this case if it were to reappear.
    Stage& rfilter = *(factory.createStage("filters.randomize"));
    rfilter.setInput(r);

    std::vector<unsigned int> kvals = {1};
    for (auto &k : kvals) {

        Options fo;
        fo.add("candidate", Support::datapath("las/sample_c_thin.las"));
        fo.add("k", k);

        Stage& f = *(factory.createStage("filters.neighborclassifier"));
        f.setInput(rfilter);
        f.setOptions(fo);

        PointTable table;
        f.prepare(table);
        PointViewSet viewSet = f.execute(table);
        PointViewPtr view = *viewSet.begin();

        EXPECT_EQ(1u, viewSet.size());
        EXPECT_EQ(count, view->size());

        stats::Summary::EnumMap NewClassifications = GetClassifications(f);

        for (auto& p : OrigClassifications)
        {
            if (p.first == 6)
                EXPECT_TRUE(NewClassifications[p.first] == 12441 && OrigClassifications[p.first] == 12525);
        }
    }
}

} // namespace pdal
