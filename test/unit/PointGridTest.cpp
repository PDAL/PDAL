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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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
#include "Support.hpp"

#include <io/LasReader.hpp>
#include <filters/CropFilter.hpp>
#include <pdal/private/PointGrid.hpp>
#include <pdal/KDIndex.hpp>

using namespace pdal;

void gridTest(std::string filename, int knn, int viewPos, Stage* prefilter = nullptr)
{
    class TestFilter : public Filter
    {
    public:
        TestFilter(std::string mode, int knn, const PointId pos) 
            : m_mode(mode), m_knn(knn), m_pos(pos)
        {}

        std::string getName() const
        {
            return "filters.testfilter";
        }
    private:
        std::string m_mode;
        int m_knn;
        PointId m_pos;
        Dimension::Id distDim;

        void addDimensions(PointLayoutPtr layout)
        {
            distDim = layout->assignDim("knn_distance", Dimension::Type::Double);
        }

        void filter(PointView& view)
        {
            if (m_mode == "pointgrid")
            {
                BOX2D box;
                view.calculateBounds(box);
                PointGrid grid(box, view, 30);
                PointGrid::NeighborResults neighbors = 
                    grid.findKnn({view.getFieldAs<double>(Dimension::Id::X, m_pos), 
                        view.getFieldAs<double>(Dimension::Id::Y, m_pos)}, m_knn);

                for (PointId i = 0; i < neighbors.size(); ++i)
                    view.setField(distDim, i, neighbors[i].second);
            }
            else if (m_mode == "kdtree")
            {
                const KD2Index& index = view.build2dIndex();
                PointIdList ids;
                std::vector<double> sqr_dists;
                index.knnSearch(m_pos, m_knn, &ids, &sqr_dists);
                for (PointId i = 0; i < ids.size(); ++i)
                    view.setField(distDim, i, sqr_dists[i]);
            }
        }
    };
    
    LasReader r;
    Options ro;
    ro.add("filename", filename);
    r.setOptions(ro);
    if (prefilter)
        prefilter->setInput(r);

    TestFilter pgFilter("pointgrid", 100, 800);
    if (prefilter)
        pgFilter.setInput(*prefilter);
    else
        pgFilter.setInput(r);
    PointTable table;
    pgFilter.prepare(table);
    PointViewSet viewSet = pgFilter.execute(table);
    PointViewPtr pgView = *viewSet.begin();

    TestFilter kdFilter("kdtree", 100, 800);
    if (prefilter)
        kdFilter.setInput(*prefilter);
    else
        kdFilter.setInput(r);
    PointTable table2;
    kdFilter.prepare(table2);
    viewSet = kdFilter.execute(table);
    PointViewPtr kdView = *viewSet.begin();

    //EXPECT_EQ(pgView->size(), kdView->size());
    Dimension::Id distDim = pgView->layout()->findDim("knn_distance");
    for (PointId i = 0; i < pgView->size(); ++i)
    {
        EXPECT_NEAR(pgView->getFieldAs<double>(distDim, i), 
            kdView->getFieldAs<double>(distDim, i), 0.0001);
    }
}

TEST(PointGridTest, largeSet)
{
    CropFilter f;
    Options fo;
    fo.add("polygon", "POLYGON ((1639605.52151933 1454498.88572908,1639605.20941558 1454507.23450445,1639597.48484772 1454506.45424507,1639595.2089086 1454685.11546591,1639776.14606856 1454683.97749635,1639770.45622077 1454489.38470167,1639605.52151933 1454498.88572908))");
    fo.add("outside", true);
    f.setOptions(fo);

    gridTest(Support::datapath("las/autzen_trim.las"), 100, 0, &f);
}