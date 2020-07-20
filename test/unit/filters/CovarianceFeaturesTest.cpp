/******************************************************************************
* Copyright (c) 2019, Helix Re Inc. nicolas@helix.re
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
*     * Neither the name of Helix Re Inc. nor the
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
#include <io/BufferReader.hpp>
#include <io/TextReader.hpp>
#include <filters/CovarianceFeaturesFilter.hpp>

#include "Support.hpp"

namespace pdal {

TEST(DimensionalityTest, Linearity)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;
    CovarianceFeaturesFilter filter;
    Options ops;
    ops.add("knn", 3);
    ops.add("feature_set", "Dimensionality,Omnivariance,Anisotropy,Eigenentropy,EigenvalueSum,SurfaceVariation,DemantkeVerticality");
    filter.setInput(bufferReader);
    filter.setOptions(ops);
    filter.prepare(table);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0,0);
    view->setField(Id::X, 1, 0);
    view->setField(Id::X, 2, 0);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 0);
    view->setField(Id::Y, 2, 0);
    view->setField(Id::Z, 0, 0);
    view->setField(Id::Z, 1, 1);
    view->setField(Id::Z, 2, 2);
    bufferReader.addView(view);

    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    for (point_count_t i =0; i < outView->size(); i++)
    {
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Linearity, i) ,1);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Verticality, i) ,1);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Planarity, i) ,0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Scattering, i) ,0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Omnivariance, i), 0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Anisotropy, i), 1);
        ASSERT_TRUE(std::isnan(outView->getFieldAs<float>(Id::Eigenentropy, i)));
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::EigenvalueSum, i), 1);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::SurfaceVariation, i), 0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::DemantkeVerticality, i), 1);
    }
}

TEST(DimensionalityTest, Planarity)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;
    CovarianceFeaturesFilter filter;
    Options ops;
    ops.add("feature_set", "Dimensionality,Omnivariance,Anisotropy,Eigenentropy,EigenvalueSum,SurfaceVariation,DemantkeVerticality");
    filter.setInput(bufferReader);
    filter.setOptions(ops);
    filter.prepare(table);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0,0);
    view->setField(Id::X, 1, 1);
    view->setField(Id::X, 2, 1);
    view->setField(Id::X, 3, 0);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 0);
    view->setField(Id::Y, 2, 1);
    view->setField(Id::Y, 3, 1);
    view->setField(Id::Z, 0, 0);
    view->setField(Id::Z, 1, 0);
    view->setField(Id::Z, 2, 0);
    view->setField(Id::Z, 3, 0);

    bufferReader.addView(view);


    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    for (point_count_t i =0; i < outView->size(); i++)
    {
        ASSERT_LE(outView->getFieldAs<float>(Id::Linearity, i) ,0.5);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Verticality, i) ,0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Planarity, i) ,1);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Scattering, i) ,0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Omnivariance, i), 0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::Anisotropy, i), 1);
        ASSERT_TRUE(std::isnan(outView->getFieldAs<float>(Id::Eigenentropy, i)));
        ASSERT_NEAR(outView->getFieldAs<float>(Id::EigenvalueSum, i), 0.667, 0.001);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::SurfaceVariation, i), 0);
        ASSERT_FLOAT_EQ(outView->getFieldAs<float>(Id::DemantkeVerticality, i), 0);
    }
}

TEST(DimensionalityTest, Scattering)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;
    CovarianceFeaturesFilter filter;
    Options ops;
    ops.add("feature_set", "Dimensionality,Omnivariance,Anisotropy,Eigenentropy,EigenvalueSum,SurfaceVariation,DemantkeVerticality");
    filter.setInput(bufferReader);
    filter.setOptions(ops);
    filter.prepare(table);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0,0);
    view->setField(Id::X, 1, 1);
    view->setField(Id::X, 2, 1);
    view->setField(Id::X, 3, 0);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 0);
    view->setField(Id::Y, 2, 1);
    view->setField(Id::Y, 3, 1);
    view->setField(Id::Z, 0, 0);
    view->setField(Id::Z, 1, 0);
    view->setField(Id::Z, 2, 1);
    view->setField(Id::Z, 3, 2);

    bufferReader.addView(view);


    PointViewSet viewSet = filter.execute(table);
    PointViewPtr outView = *viewSet.begin();

    for (point_count_t i =0; i < outView->size(); i++)
    {
        ASSERT_LE(outView->getFieldAs<float>(Id::Linearity, i) ,0.5);
        ASSERT_GE(outView->getFieldAs<float>(Id::Verticality, i) ,0.5);
        ASSERT_LE(outView->getFieldAs<float>(Id::Planarity, i) ,1);
        ASSERT_GE(outView->getFieldAs<float>(Id::Scattering, i) ,0.1);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::Omnivariance, i), 0.458, 0.001);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::Anisotropy, i), 0.864, 0.001);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::Eigenentropy, i), 0.488, 0.001);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::EigenvalueSum, i), 1.583, 0.001);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::SurfaceVariation, i), 0.095, 0.001);
        ASSERT_NEAR(outView->getFieldAs<float>(Id::DemantkeVerticality, i), 0.492, 0.001);
    }
}

}
