/******************************************************************************
 * Copyright (c) 2019, Connor Manning (connor@hobu.co)
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

#include <pdal/pdal_test_main.hpp>

#include <nlohmann/json.hpp>

#include <pdal/util/FileUtils.hpp>
#include <filters/AssignFilter.hpp>
#include <filters/FerryFilter.hpp>
#include <io/EptReader.hpp>
#include <io/EptAddonWriter.hpp>
#include <io/LasReader.hpp>
#include "Support.hpp"

using namespace pdal;

namespace
{
    const std::string eptLaszipPath(
        Support::datapath("ept/lone-star-laszip/ept.json"));
}

TEST(EptAddonWriterTest, fullLoop)
{
    // Test the writing, and subsequent reading, of EPT addons from both
    // well-known and proprietary dimensions.
    const std::string addonDir(Support::datapath("ept/addon/"));
    FileUtils::deleteDirectory(addonDir);

    // First write the output.
    {
        EptReader reader;
        {
            Options o;
            o.add("filename", eptLaszipPath);
            reader.setOptions(o);
        }

        // Assign Classification values.
        AssignFilter assign1;
        {
            Options o;
            o.add("assignment", "Classification[:]=42");
            assign1.setOptions(o);
            assign1.setInput(reader);
        }

        // Ferry Classification => Other to create a new dimension that we can
        // access in the next step.
        FerryFilter ferry;
        {
            Options o;
            o.add("dimensions", "Classification => Other");
            ferry.setOptions(o);
            ferry.setInput(assign1);
        }

        // Assign proprietary dimension values.
        AssignFilter assign2;
        {
            Options o;
            o.add("assignment", "Other[:]=88");
            assign2.setOptions(o);
            assign2.setInput(ferry);
        }

        EptAddonWriter writer;
        {
            NL::json addons;
            addons[addonDir + "class"] = "Classification";
            addons[addonDir + "other"] = "Other";

            Options o;
            o.add("addons", addons);
            writer.setOptions(o);
            writer.setInput(assign2);
        }

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    // Then read the output, making sure our dimension values are overridden
    // with the addon values.
    EptReader reader;
    {
        NL::json addons;
        addons["Classification"] = addonDir + "class";
        addons["Other"] = addonDir + "other";

        Options o;
        o.add("filename", eptLaszipPath);
        o.add("addons", addons);
        reader.setOptions(o);
    }

    PointTable table;
    reader.prepare(table);
    const auto set(reader.execute(table));

    const Dimension::Id classDim(Dimension::Id::Classification);
    const Dimension::Id otherDim(table.layout()->findDim("Other"));

    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            ASSERT_EQ(view->getFieldAs<uint16_t>(classDim, i), 42u);
            ASSERT_EQ(view->getFieldAs<uint16_t>(otherDim, i), 88u);
        }
    }
}

TEST(EptAddonWriterTest, boundedWrite)
{
    // Assign the result of a bounded query.  This should write the attribute
    // for only the values selected by these bounds, leaving the relevant
    // attribute at zero for the rest of the dataset.

    const std::string addonDir(Support::datapath("ept/addon/"));
    FileUtils::deleteDirectory(addonDir);

    BOX2D bounds(515380, 4918350, 515400, 4918370);

    {
        EptReader reader;
        {
            Options o;
            o.add("filename", eptLaszipPath);
            o.add("bounds", bounds);
            reader.setOptions(o);
        }

        AssignFilter assign;
        {
            Options o;
            o.add("assignment", "Classification[:]=42");
            assign.setOptions(o);
            assign.setInput(reader);
        }

        EptAddonWriter writer;
        {
            NL::json addons;
            addons[addonDir + "bounded"] = "Classification";

            Options o;
            o.add("addons", addons);
            writer.setOptions(o);
            writer.setInput(assign);
        }

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    // Now we'll query the whole dataset with this addon - points outside the
    // bounds should have a Classification of zero.

    EptReader reader;
    {
        NL::json addons;
        addons["Classification"] = addonDir + "bounded";

        Options o;
        o.add("filename", eptLaszipPath);
        o.add("addons", addons);
        reader.setOptions(o);
    }

    PointTable table;
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y;
    uint8_t c;

    int in(0), out(0);

    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            c = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);

            EXPECT_GT(x, 0);
            EXPECT_GT(y, 0);
            if (bounds.contains(x, y))
            {
                ++in;
                ASSERT_EQ(c, 42u);
            }
            else
            {
                ++out;
                ASSERT_EQ(c, 0u);
            }
        }
    }

    // Make sure our bounds are actually selecting data and pruning
    // appropriately.
    EXPECT_GT(in, 0);
    EXPECT_GT(out, 0);
}

TEST(EptAddonWriterTest, boundedRead)
{
    // Make sure that when we query an EPT set using a boundary that it
    // looks consistent.

    const std::string addonDir(Support::datapath("ept/addon/"));
    FileUtils::deleteDirectory(addonDir);

    {
        EptReader reader;
        {
            Options o;
            o.add("filename", eptLaszipPath);
            reader.setOptions(o);
        }

        AssignFilter assign;
        {
            Options o;
            o.add("assignment", "Classification[:]=42");
            assign.setOptions(o);
            assign.setInput(reader);
        }

        EptAddonWriter writer;
        {
            NL::json addons;
            addons[addonDir + "bounded"] = "Classification";

            Options o;
            o.add("addons", addons);
            writer.setOptions(o);
            writer.setInput(assign);
        }

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    // Now we'll query the whole dataset with this addon - points outside the
    // bounds should have a Classification of zero.

    BOX2D bounds(515380, 4918300, 515450, 4918370);
    EptReader reader;
    {
        NL::json addons;
        addons["Classification"] = addonDir + "bounded";

        Options o;
        o.add("filename", eptLaszipPath);
        o.add("bounds", bounds);
        o.add("addons", addons);
        reader.setOptions(o);
    }

    PointTable table;
    reader.prepare(table);
    const auto set(reader.execute(table));

    double x, y;
    uint8_t c;

    int in(0), out(0);

    for (const PointViewPtr& view : set)
    {
        for (point_count_t i(0); i < view->size(); ++i)
        {
            x = view->getFieldAs<double>(Dimension::Id::X, i);
            y = view->getFieldAs<double>(Dimension::Id::Y, i);
            c = view->getFieldAs<uint8_t>(Dimension::Id::Classification, i);

            EXPECT_GT(x, 0);
            EXPECT_GT(y, 0);
            if (bounds.contains(x, y))
            {
                ++in;
                ASSERT_EQ(c, 42u);
            }
            else
            {
                ++out;
                ASSERT_EQ(c, 0u);
            }
        }
    }

    // Make sure our bounds are actually selecting data and pruning
    // appropriately.
    EXPECT_GT(in, 0);
    EXPECT_EQ(out, 0);
}

TEST(EptAddonWriterTest, mustDescendFromEptReader)
{
    // Make sure the EPT writer throws if it is not used in tandem with an EPT
    // reader.

    LasReader reader;
    {
        Options o;
        o.add("filename", Support::datapath("las/simple.las"));
        reader.setOptions(o);
    }

    EptAddonWriter writer;
    {
        Options o;
        NL::json addons;
        addons[Support::datapath("ept/addon/bad")] = "ReturnNumber";
        o.add("addons", addons);
        writer.setOptions(o);
        writer.setInput(reader);
    }

    PointTable table;
    writer.prepare(table);

    EXPECT_THROW(writer.execute(table), pdal_error);
}

