/******************************************************************************
 * Copyright (c) 2018, Connor Manning (connor@hobu.co)
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

#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include "json/json.h"

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/ExpressionFilter.hpp>

using namespace pdal;

namespace
{

using D = Dimension::Id;
const Dimension::IdList dims { { D::X, D::Y, D::Z } };

std::unique_ptr<FixedPointTable> makeTable()
{
    std::unique_ptr<FixedPointTable> table(new FixedPointTable(1));
    table->layout()->registerDims(dims);
    table->finalize();
    return table;
}

std::unique_ptr<ExpressionFilter> makeFilter(BasePointTable& table,
        Json::Value expression)
{
    Options o;
    o.add("expression", expression.toStyledString());
    std::unique_ptr<ExpressionFilter> filter(new ExpressionFilter());
    filter->setOptions(o);
    filter->prepare(table);
    return filter;
}

} // unnamed namespace

TEST(ExpressionFilterTest, createStage)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.expression"));
    EXPECT_TRUE(filter);
}

TEST(ExpressionFilterTest, noExpression)
{
    ExpressionFilter filter;
    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(ExpressionFilterTest, missingDimension)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Missing LHS dimension.
        Json::Value e;
        e["Red"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    {
        // Missing RHS dimension.
        Json::Value e;
        e["X"] = "Red";
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(ExpressionFilterTest, invalidSingleComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Comparison operators must take values, not arrays.
        Json::Value e;
        e["X"]["$eq"].append(1);
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    {
        // Comparison operators must take values, not objects.
        Json::Value e;
        e["X"]["$eq"]["asdf"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(ExpressionFilterTest, singleComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Implicit $eq.
        Json::Value e;
        e["X"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));

        // Across dimensions.
        e["X"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }

    {
        // Explicit $eq.
        Json::Value e;
        e["X"]["$eq"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$eq"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }

    {
        // $ne
        Json::Value e;
        e["X"]["$ne"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$ne"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

    }

    {
        // $gt
        Json::Value e;
        e["X"]["$gt"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$gt"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

    }

    {
        // $gte
        Json::Value e;
        e["X"]["$gte"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$gte"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));
    }

    {
        // $lt
        Json::Value e;
        e["X"]["$lt"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$lt"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }

    {
        // $lte
        Json::Value e;
        e["X"]["$lte"] = 0;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));

        // Across dimensions.
        e["X"]["$lte"] = "Y";
        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }
}

TEST(ExpressionFilterTest, inValidMultiComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Comparison operators must take arrays, not values.
        Json::Value e;
        e["X"]["$in"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
    {
        // Comparison operators must take arrays, not objects.
        Json::Value e;
        e["X"]["$in"]["asdf"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
    {
        // Dimensions must exist.
        Json::Value e;
        e["X"]["$in"].append("Red");
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(ExpressionFilterTest, multiComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    // $in.
    {
        Json::Value e;
        e["X"]["$in"].append(0);
        e["X"]["$in"].append(1);
        e["X"]["$in"].append(2);
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_FALSE(f->processOne(pr));
    }

    // $in across dimensions.
    {
        Json::Value e;
        e["X"]["$in"].append(0);
        e["X"]["$in"].append(1);
        e["X"]["$in"].append("Y");
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::Y, 2);

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_FALSE(f->processOne(pr));
    }

    // $nin.
    {
        Json::Value e;
        e["X"]["$nin"].append(0);
        e["X"]["$nin"].append(1);
        e["X"]["$nin"].append(2);
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_TRUE(f->processOne(pr));
    }

    // $nin across dimensions.
    {
        Json::Value e;
        e["X"]["$nin"].append(0);
        e["X"]["$nin"].append(1);
        e["X"]["$nin"].append("Y");
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::Y, 2);

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_TRUE(f->processOne(pr));
    }
}

TEST(ExpressionFilterTest, invalidLogicalOperators)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    // Logical operators cannot point to values.
    {
        Json::Value e;
        e["$and"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical operators cannot point to objects.
    {
        Json::Value e;
        e["$and"]["X"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical NOT is an oddball compared to the other logical operators, which
    // take arrays.  Logical NOT accepts a single expression which it negates.

    // Logical NOT cannot point to values.
    {
        Json::Value e;
        e["$not"] = 42;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical NOT must accept only a single expression.
    {
        Json::Value e;
        Json::Value arr;
        {
            Json::Value curr;
            curr["X"] = 0;
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Y"] = 1;
            arr.append(curr);
        }

        e["$not"] = arr;
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(ExpressionFilterTest, logicalOperators)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    std::array<double, 3> vals { { 0, 1, 2 } };

    // Implicit $and.
    {
        Json::Value e;
        e["X"]["$gt"] = 0;
        e["X"]["$lt"] = 2;
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));
    }

    // Implicit $and across dimensions.
    {
        Json::Value e;
        e["X"]["$gt"] = 0;
        e["X"]["$lt"] = "Y";
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::Y, 2);

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));
    }

    // $and.
    {
        Json::Value e;
        Json::Value arr;
        {
            Json::Value curr;
            curr["X"] = vals[0];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Y"] = vals[1];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Z"] = vals[2];
            arr.append(curr);
        }

        e["$and"] = arr;

        auto f(makeFilter(*table, e));

        for (PointId x(0); x < 3; ++x)
        {
            for (PointId y(0); y < 3; ++y)
            {
                for (PointId z(0); z < 3; ++z)
                {
                    pr.setField(Dimension::Id::X, x);
                    pr.setField(Dimension::Id::Y, y);
                    pr.setField(Dimension::Id::Z, z);

                    bool check(x == vals[0] && y == vals[1] && z == vals[2]);
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $or.
    {
        Json::Value e;
        Json::Value arr;
        {
            Json::Value curr;
            curr["X"] = vals[0];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Y"] = vals[1];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Z"] = vals[2];
            arr.append(curr);
        }

        e["$or"] = arr;

        auto f(makeFilter(*table, e));

        for (PointId x(0); x < 3; ++x)
        {
            for (PointId y(0); y < 3; ++y)
            {
                for (PointId z(0); z < 3; ++z)
                {
                    pr.setField(Dimension::Id::X, x);
                    pr.setField(Dimension::Id::Y, y);
                    pr.setField(Dimension::Id::Z, z);

                    bool check(x == vals[0] || y == vals[1] || z == vals[2]);
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $nor.
    {
        Json::Value e;
        Json::Value arr;
        {
            Json::Value curr;
            curr["X"] = vals[0];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Y"] = vals[1];
            arr.append(curr);
        }
        {
            Json::Value curr;
            curr["Z"] = vals[2];
            arr.append(curr);
        }

        e["$nor"] = arr;

        auto f(makeFilter(*table, e));

        for (PointId x(0); x < 3; ++x)
        {
            for (PointId y(0); y < 3; ++y)
            {
                for (PointId z(0); z < 3; ++z)
                {
                    pr.setField(Dimension::Id::X, x);
                    pr.setField(Dimension::Id::Y, y);
                    pr.setField(Dimension::Id::Z, z);

                    bool check(!(x == vals[0] || y == vals[1] || z == vals[2]));
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $not
    {
        Json::Value e;
        e["$not"]["X"]["$gt"] = 0;

        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }

    // $not with inner multi-comparison
    {
        Json::Value e;
        e["$not"]["X"]["$in"].append(0);
        e["$not"]["X"]["$in"].append(1);
        e["$not"]["X"]["$in"].append(2);

        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_TRUE(f->processOne(pr));
    }
}

