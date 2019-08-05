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

#include <nlohmann/json.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/MongoExpressionFilter.hpp>

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

std::unique_ptr<MongoExpressionFilter> makeFilter(BasePointTable& table,
    NL::json expression)
{
    Options o;
    o.add("expression", expression.dump());
    std::unique_ptr<MongoExpressionFilter> filter(new MongoExpressionFilter());
    filter->setOptions(o);
    filter->prepare(table);
    return filter;
}

} // unnamed namespace

TEST(MongoExpressionFilterTest, createStage)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.mongo"));
    EXPECT_TRUE(filter);
}

TEST(MongoExpressionFilterTest, noExpression)
{
    MongoExpressionFilter filter;
    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(MongoExpressionFilterTest, missingDimension)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Missing LHS dimension.
        NL::json e {"Red", 42};
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    {
        // Missing RHS dimension.
        NL::json e {"X", "Red"};
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(MongoExpressionFilterTest, invalidSingleComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Comparison operators must take values, not arrays.
        NL::json e {"X", {"$eq", {1}} };
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    {
        // Comparison operators must take values, not objects.
        NL::json e {"X", {"$eq", {"asdf", 42} } };
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(MongoExpressionFilterTest, singleComparisons)
{
    auto table(makeTable());

    {
        // Implicit $eq.
        NL::json e { { "X", 0 } };

        auto f(makeFilter(*table, e));
        PointRef pr(*table, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }
    {
        // Across dimensions.
        NL::json e { {"X", "Y"} };
        auto f(makeFilter(*table, e));
        PointRef pr(*table, 0);

        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_TRUE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 1);
        EXPECT_FALSE(f->processOne(pr));
    }


    auto constant = [&table](const std::string& comp,
        bool neg, bool zero, bool pos)
    {
        PointRef pr(*table, 0);

        NL::json e {{"X", {{comp, 0}} }};
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, -1);
        EXPECT_EQ(f->processOne(pr), neg) << "Error comparing to 0 "
            "with comparator " << comp << " and negative test.";

        pr.setField(Dimension::Id::X, 0);
        EXPECT_EQ(f->processOne(pr), zero) << "Error comparing to 0 "
            "with comparator " << comp << " and zero test.";

        pr.setField(Dimension::Id::X, 1);
        EXPECT_EQ(f->processOne(pr), pos) << "Error comparing to 0 "
            "with comparator " << comp << " and positive test.";
    };

    constant("$eq", false, true, false);
    constant("$ne", true, false, true);
    constant("$gt", false, false, true);
    constant("$gte", false, true, true);
    constant("$lt", true, false, false);
    constant("$lte", true, true, false);

    auto across = [&table](const std::string& comp,
        bool neg, bool zero, bool pos)
    {
        PointRef pr(*table, 0);

        NL::json e {{"X", {{comp, "Y"}} }};
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::Y, 0);

        pr.setField(Dimension::Id::X, -1);
        EXPECT_EQ(f->processOne(pr), neg) << "Error comparing across "
            "dimensions with comparator " << comp << " and negative test.";

        pr.setField(Dimension::Id::X, 0);
        EXPECT_EQ(f->processOne(pr), zero) << "Error comparing across "
            "dimensions with comparator " << comp << " and zero test.";

        pr.setField(Dimension::Id::X, 1);
        EXPECT_EQ(f->processOne(pr), pos) << "Error comparing across "
            "dimensions with comparator " << comp << " and positive test.";
    };

    across("$eq", false, true, false);
    across("$ne", true, false, true);
    across("$gt", false, false, true);
    across("$gte", false, true, true);
    across("$lt", true, false, false);
    across("$lte", true, true, false);
}

TEST(MongoExpressionFilterTest, invalidMultiComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    {
        // Comparison operators must take arrays, not values.
        NL::json e {"X", {"$in", 42} };
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
    {
        // Comparison operators must take arrays, not objects.
        NL::json e {"X", {"$in", {"asdf", 42} } };
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
    {
        // Dimensions must exist.
        NL::json e {"X", {"$in", {"Red"}} };
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(MongoExpressionFilterTest, multiComparisons)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    // $in.
    {
        NL::json e {{ "X", {{"$in", {0, 1, 2} }} }};
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
        NL::json e {{ "X", {{"$in", {0, 1, "Y"} }} }};
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
        NL::json e {{ "X", {{"$nin", {0, 1, 2} }} }};
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
        NL::json e {{ "X", {{"$nin", {0, 1, "Y"} }} }};
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

TEST(MongoExpressionFilterTest, invalidLogicalOperators)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    // Logical operators cannot point to values.
    {
        NL::json e {"$and", 42};
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical operators cannot point to objects.
    {
        NL::json e {"$and", "X"};
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical NOT is an oddball compared to the other logical operators, which
    // take arrays.  Logical NOT accepts a single expression which it negates.

    // Logical NOT cannot point to values.
    {
        NL::json e {"$not", 42};
        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }

    // Logical NOT must accept only a single expression.
    {
        NL::json e {"$not", {{"X", 0}, {"Y", 1}} };

        EXPECT_THROW(makeFilter(*table, e), pdal_error);
    }
}

TEST(MongoExpressionFilterTest, logicalOperators)
{
    auto table(makeTable());
    PointRef pr(*table, 0);

    // Implicit $and.
    {
        NL::json e {{"X", {{"$gt", 0}, {"$lt", 2}} }};
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
        NL::json e {{"X", {{"$gt", 0}, {"$lt", "Y"}} }};
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
        NL::json e {{"$and", { {{"X", 0}}, {{"Y", 1}}, {{"Z", 2}} } }};

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

                    bool check(x == 0 && y == 1 && z == 2);
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $or.
    {
        NL::json e {{"$or", { {{"X", 0}}, {{"Y", 1}}, {{"Z", 2}} } }};
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

                    bool check(x == 0 || y == 1 || z == 2);
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $nor.
    {
        NL::json e {{"$nor", { {{"X", 0}}, {{"Y", 1}}, {{"Z", 2}} } }};

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

                    bool check(!(x == 0 || y == 1 || z == 2));
                    EXPECT_EQ(f->processOne(pr), check) << x << ", " << y <<
                        ", " << z << " != " << check << std::endl;
                }
            }
        }
    }

    // $not
    {
        NL::json e {{"$not", {{"X", {{"$gt", 0}} }} }};
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
        NL::json e {{"$not", {{"X", {{"$in", {0, 1, 2} }} }} }};
        auto f(makeFilter(*table, e));

        pr.setField(Dimension::Id::X, 0);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 2);
        EXPECT_FALSE(f->processOne(pr));

        pr.setField(Dimension::Id::X, 4);
        EXPECT_TRUE(f->processOne(pr));
    }
}

