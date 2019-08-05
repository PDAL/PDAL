/******************************************************************************
* Copyright (c) 2018, RIEGL Laser Measurement Systems GmbH (support@riegl.com)
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
*     * Neither the name of Hobu, Inc., Flaxen Geo Consulting or RIEGL
*       Laser Measurement Systems GmbH nor the names of its contributors
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

#include <set>
#include <string>
#include <limits>

#include <pdal/pdal_test_main.hpp>

#include <pdal/Options.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PointView.hpp>

#include "RdbReader.hpp"
#include "Config.hpp"

using namespace pdal;


//---< LOCAL TOOLS >------------------------------------------------------------


namespace //anonymous
{

Options rdbReaderOptions(
    const std::string& source = "",
    const std::string& filter = "",
    const bool         extras = false
)
{
    Options options;
    const std::string name(source.empty() ? "autzen-thin-srs.rdbx" : source);
    const std::string mask(filter.empty() ? "riegl.id != 0"        : filter);
    options.add(Option("filename", testDataPath() + name));
    options.add(Option("filter",   mask));
    options.add(Option("extras",   extras));
    return options;
}

Options rdbReaderOptions(
    const std::string& filter,
    const bool         extras
)
{
    return rdbReaderOptions("", filter, extras);
}

template <typename ValueType>
void checkDimension(
    const PointViewPtr  view,
    const PointId       index,
    const Dimension::Id dimension,
    const ValueType     expected,
    const double        epsilon
)
{
    const auto title = view->dimName(dimension);
    const auto value = view->getFieldAs<ValueType>(dimension, index);
    EXPECT_NEAR(value, expected, epsilon) << "dimension: " << title;
}

void checkPoint(
    const PointViewPtr view,
    const PointId      index,
    const double       X,
    const double       Y,
    const double       Z,
    const uint16_t     Red,
    const uint16_t     Green,
    const uint16_t     Blue,
    const uint32_t     PointId,
    const uint16_t     Intensity,
    const double       InternalTime,
    const uint8_t      ReturnNumber,
    const uint8_t      NumberOfReturns,
    const uint8_t      Classification,
    const uint32_t     OriginId,
    const float        ScanAngleRank,
    const uint8_t      ScanDirectionFlag
)
{
    using namespace Dimension;
    checkDimension(view, index, Id::X,                 X,                 0.00025);
    checkDimension(view, index, Id::Y,                 Y,                 0.00025);
    checkDimension(view, index, Id::Z,                 Z,                 0.00025);
    checkDimension(view, index, Id::Red,               Red,               0);
    checkDimension(view, index, Id::Green,             Green,             0);
    checkDimension(view, index, Id::Blue,              Blue,              0);
    checkDimension(view, index, Id::PointId,           PointId,           0);
    checkDimension(view, index, Id::Intensity,         Intensity,         0);
    checkDimension(view, index, Id::InternalTime,      InternalTime,      1e-7);
    checkDimension(view, index, Id::ReturnNumber,      ReturnNumber,      0);
    checkDimension(view, index, Id::NumberOfReturns,   NumberOfReturns,   0);
    checkDimension(view, index, Id::Classification,    Classification,    0);
    checkDimension(view, index, Id::OriginId,          OriginId,          0);
    checkDimension(view, index, Id::ScanAngleRank,     ScanAngleRank,     0.006);
    checkDimension(view, index, Id::ScanDirectionFlag, ScanDirectionFlag, 0);
}

} // anonymous namespace


//---< UNIT TESTS >-------------------------------------------------------------


TEST(RdbReaderTest, Reader) // creating the reader works?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions());

    EXPECT_EQ(reader.getName(), "readers.rdb");
}


TEST(RdbReaderTest, Missing) // loading a missing file fails?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions("some-file-that-does-not-exist.rdbx"));

    PointTable table;
    EXPECT_ANY_THROW(reader.prepare(table));
}


TEST(RdbReaderTest, Metadata) // reading metadata works?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions());

    PointTable table;
    reader.prepare(table);

    const MetadataNode root = reader.getMetadata();
    EXPECT_TRUE(root.valid());
    {
        const MetadataNode database = root    .findChild("database");
        const MetadataNode uuid     = database.findChild("uuid");
        const MetadataNode points   = database.findChild("points");
        const MetadataNode bounds   = database.findChild("bounds");
        const MetadataNode minimum  = bounds  .findChild("minimum");
        const MetadataNode maximum  = bounds  .findChild("maximum");

        EXPECT_TRUE(database.valid());
        EXPECT_TRUE(uuid    .valid());
        EXPECT_TRUE(points  .valid());
        EXPECT_TRUE(bounds  .valid());
        EXPECT_TRUE(minimum .valid());
        EXPECT_TRUE(maximum .valid());

        EXPECT_EQ(
            "637de54d-7e6b-4004-b6ab-b6bc588ec9ea",
            uuid.value<std::string>()
        );
        EXPECT_EQ(
            size_t(10653),
            points.value<size_t>()
        );
        {
            const MetadataNodeList xyz = minimum.children();
            EXPECT_EQ(size_t(3), xyz.size());
            EXPECT_NEAR(-2505882.45900, xyz[0].value<double>(), 0.00025);
            EXPECT_NEAR(-3848231.39300, xyz[1].value<double>(), 0.00025);
            EXPECT_NEAR( 4412172.54800, xyz[2].value<double>(), 0.00025);
        }
        {
            const MetadataNodeList xyz = maximum.children();
            EXPECT_EQ(size_t(3), xyz.size());
            EXPECT_NEAR(-2504493.76200, xyz[0].value<double>(), 0.00025);
            EXPECT_NEAR(-3846841.25200, xyz[1].value<double>(), 0.00025);
            EXPECT_NEAR( 4413210.39400, xyz[2].value<double>(), 0.00025);
        }
    }
    {
        const MetadataNodeList transactions = root.children("transactions");
        EXPECT_EQ(size_t(5), transactions.size());

        std::set<std::string> ids;
        for (const auto& transaction: transactions)
        {
            EXPECT_TRUE(transaction.findChild("id")      .valid());
            EXPECT_TRUE(transaction.findChild("rdb")     .valid());
            EXPECT_TRUE(transaction.findChild("title")   .valid());
            EXPECT_TRUE(transaction.findChild("agent")   .valid());
            EXPECT_TRUE(transaction.findChild("comments").valid());
            EXPECT_TRUE(transaction.findChild("settings").valid());
            EXPECT_TRUE(transaction.findChild("start")   .valid());
            EXPECT_TRUE(transaction.findChild("stop")    .valid());
            ids.insert (transaction.findChild("id").value<std::string>());
        }
        EXPECT_EQ(size_t(1), ids.count("1"));
        EXPECT_EQ(size_t(1), ids.count("2"));
        EXPECT_EQ(size_t(1), ids.count("3"));
        EXPECT_EQ(size_t(1), ids.count("4"));
        EXPECT_EQ(size_t(1), ids.count("5"));
    }
    {
        const MetadataNodeList dimensions = root.children("dimensions");
        EXPECT_EQ(size_t(14), dimensions.size());

        std::set<std::string> names;
        for (const auto& dimension: dimensions)
        {
            EXPECT_TRUE (dimension.findChild("name")               .valid());
            EXPECT_TRUE (dimension.findChild("title")              .valid());
            EXPECT_TRUE (dimension.findChild("description")        .valid());
            EXPECT_TRUE (dimension.findChild("unit_symbol")        .valid());
            EXPECT_TRUE (dimension.findChild("length")             .valid());
            EXPECT_TRUE (dimension.findChild("resolution")         .valid());
            EXPECT_TRUE (dimension.findChild("minimum_value")      .valid());
            EXPECT_TRUE (dimension.findChild("maximum_value")      .valid());
            EXPECT_TRUE (dimension.findChild("default_value")      .valid());
//             EXPECT_TRUE (dimension.findChild("invalid_value")      .valid());
            EXPECT_TRUE (dimension.findChild("storage_class")      .valid());
            EXPECT_TRUE (dimension.findChild("compression_options").valid());
            EXPECT_TRUE (dimension.findChild("scale_factor")       .valid());
            names.insert(dimension.findChild("name").value<std::string>());
        }
        EXPECT_EQ(size_t(1), names.count("riegl.id"));
        EXPECT_EQ(size_t(1), names.count("riegl.xyz"));
        EXPECT_EQ(size_t(1), names.count("riegl.timestamp"));
        EXPECT_EQ(size_t(1), names.count("riegl.intensity"));
        EXPECT_EQ(size_t(1), names.count("riegl.target_index"));
        EXPECT_EQ(size_t(1), names.count("riegl.target_count"));
        EXPECT_EQ(size_t(1), names.count("riegl.source_cloud_id"));
        EXPECT_EQ(size_t(1), names.count("riegl.class"));
        EXPECT_EQ(size_t(1), names.count("riegl.rgba"));
        EXPECT_EQ(size_t(1), names.count("riegl.selected"));
        EXPECT_EQ(size_t(1), names.count("riegl.visible"));
        EXPECT_EQ(size_t(1), names.count("riegl.end_of_scan_line"));
        EXPECT_EQ(size_t(1), names.count("riegl.scan_direction"));
        EXPECT_EQ(size_t(1), names.count("riegl.scan_angle"));
    }
    {
        const MetadataNode metadata = root.findChild("metadata");
        EXPECT_GT(metadata.children().size(), size_t(0));

        const MetadataNode     geo  = metadata.findChild("riegl.geo_tag");
        const MetadataNode     crs  = geo.findChild("crs");
        const MetadataNode     epsg = crs.findChild("epsg");
        const MetadataNode     wkt  = crs.findChild("wkt");
        const MetadataNodeList pose = geo.children("pose");

        EXPECT_EQ(size_t(4956), epsg.value<size_t>());
        EXPECT_GT(wkt.value<std::string>().size(), size_t(100));

        EXPECT_EQ(size_t(16), pose.size());
        EXPECT_NEAR( 0.837957447, pose[ 0].value<double>(), 1e-9);
        EXPECT_NEAR( 0.379440385, pose[ 1].value<double>(), 1e-9);
        EXPECT_NEAR(-0.392240121, pose[ 2].value<double>(), 1e-9);
        EXPECT_NEAR(-2505819.156, pose[ 3].value<double>(), 1e-4);
        EXPECT_NEAR(-0.545735575, pose[ 4].value<double>(), 1e-9);
        EXPECT_NEAR( 0.582617132, pose[ 5].value<double>(), 1e-9);
        EXPECT_NEAR(-0.602270669, pose[ 6].value<double>(), 1e-9);
        EXPECT_NEAR(-3847595.645, pose[ 7].value<double>(), 1e-4);
        EXPECT_NEAR( 0.000000000, pose[ 8].value<double>(), 1e-9);
        EXPECT_NEAR( 0.718736580, pose[ 9].value<double>(), 1e-9);
        EXPECT_NEAR( 0.695282481, pose[10].value<double>(), 1e-9);
        EXPECT_NEAR( 4412064.882, pose[11].value<double>(), 1e-4);
        EXPECT_NEAR( 0.000000000, pose[12].value<double>(), 1e-9);
        EXPECT_NEAR( 0.000000000, pose[13].value<double>(), 1e-9);
        EXPECT_NEAR( 0.000000000, pose[14].value<double>(), 1e-9);
        EXPECT_NEAR( 1.000000000, pose[15].value<double>(), 1e-4);
    }
}


TEST(RdbReaderTest, Dimensions) // reading point dimensions works?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions());

    PointTable table;
    reader.prepare(table);

    PointLayoutPtr layout = table.layout();
    EXPECT_EQ(size_t(15), layout->dims().size());
    EXPECT_TRUE(layout->hasDim(Dimension::Id::X));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Y));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Z));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Red));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Green));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Blue));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::PointId));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Intensity));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::InternalTime));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::ReturnNumber));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::NumberOfReturns));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::Classification));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::OriginId));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::ScanAngleRank));
    EXPECT_TRUE(layout->hasDim(Dimension::Id::ScanDirectionFlag));
}


TEST(RdbReaderTest, Points) // reading all points works?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions());

    PointTable table;
    reader.prepare(table);

    PointViewSet views = reader.execute(table);
    EXPECT_EQ(size_t(1), views.size());
    PointViewPtr view = *views.begin();
    EXPECT_EQ(size_t(10653), view->size());

    checkPoint(
        /* view              */ view,
        /* index             */ 0,
        /* X                 */ -2505857.9689326435,
        /* Y                 */ -3847653.2263720199,
        /* Z                 */  4412180.8010971518,
        /* Red               */ 142,
        /* Green             */ 139,
        /* Blue              */ 97,
        /* PointId           */ 244,
        /* Intensity         */ 155,
        /* InternalTime      */ 246093.3299375,
        /* ReturnNumber      */ 1,
        /* NumberOfReturns   */ 1,
        /* Classification    */ 2,
        /* OriginId          */ 7327,
        /* ScanAngleRank     */ -7.998,
        /* ScanDirectionFlag */ 1
    );
    checkPoint(
        /* view              */ view,
        /* index             */ 37,
        /* X                 */ -2505814.4218817758,
        /* Y                 */ -3847580.2774434495,
        /* Z                 */  4412266.7581033576,
        /* Red               */ 56,
        /* Green             */ 73,
        /* Blue              */ 69,
        /* PointId           */ 192,
        /* Intensity         */ 42,
        /* InternalTime      */ 245388.6699714,
        /* ReturnNumber      */ 2,
        /* NumberOfReturns   */ 2,
        /* Classification    */ 1,
        /* OriginId          */ 7326,
        /* ScanAngleRank     */ -15.000,
        /* ScanDirectionFlag */ 0
    );
    checkPoint(
        /* view              */ view,
        /* index             */ 8444,
        /* X                 */ -2505031.6910822857,
        /* Y                 */ -3847301.9011413464,
        /* Z                 */  4412956.6680581868,
        /* Red               */ 126,
        /* Green             */ 130,
        /* Blue              */ 104,
        /* PointId           */ 7686,
        /* Intensity         */ 1,
        /* InternalTime      */ 248287.9177593,
        /* ReturnNumber      */ 1,
        /* NumberOfReturns   */ 2,
        /* Classification    */ 1,
        /* OriginId          */ 7331,
        /* ScanAngleRank     */ 13.002,
        /* ScanDirectionFlag */ 0
    );
    checkPoint(
        /* view              */ view,
        /* index             */ 10652,
        /* X                 */ -2504502.5481943958,
        /* Y                 */ -3847396.115774862,
        /* Z                 */  4413180.3611096218,
        /* Red               */ 62,
        /* Green             */ 78,
        /* Blue              */ 76,
        /* PointId           */ 10456,
        /* Intensity         */ 15,
        /* InternalTime      */ 249764.0242360,
        /* ReturnNumber      */ 2,
        /* NumberOfReturns   */ 2,
        /* Classification    */ 1,
        /* OriginId          */ 7334,
        /* ScanAngleRank     */ 7.998,
        /* ScanDirectionFlag */ 1
    );
}


TEST(RdbReaderTest, Filter) // point filter works?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions("riegl.id <= 100", false));

    PointTable table;
    reader.prepare(table);

    PointViewSet views = reader.execute(table);
    EXPECT_EQ(size_t(1), views.size());
    PointViewPtr view = *views.begin();
    EXPECT_EQ(size_t(100), view->size());

    std::set<size_t> pids;
    for (size_t i = 0; i < view->size(); ++i)
    {
        const auto pid = view->getFieldAs<size_t>(Dimension::Id::PointId, i);
        EXPECT_EQ(size_t(0),   pids.count(pid));
        EXPECT_GT(size_t(101), pid);
        EXPECT_LT(size_t(0),   pid);
        pids.insert(pid);
    }
    EXPECT_EQ(size_t(100), pids.size());
}


TEST(RdbReaderTest, Extras) // extra point dimensions work?
{
    RdbReader reader;
    reader.setOptions(rdbReaderOptions("riegl.id <= 100", true));

    PointTable table;
    reader.prepare(table);

    PointViewSet views = reader.execute(table);
    EXPECT_EQ(size_t(1), views.size());
    PointViewPtr view = *views.begin();
    EXPECT_EQ(size_t(100), view->size());

    PointLayoutPtr layout = table.layout();
    EXPECT_EQ(size_t(18), layout->dims().size());

    const auto riegl_visible     = layout->findDim("riegl.visible");
    const auto riegl_selected    = layout->findDim("riegl.selected");
    const auto riegl_end_of_line = layout->findDim("riegl.end_of_scan_line");

    EXPECT_TRUE(layout->hasDim(riegl_visible));
    EXPECT_TRUE(layout->hasDim(riegl_selected));
    EXPECT_TRUE(layout->hasDim(riegl_end_of_line));

    EXPECT_EQ(Dimension::Type::Unsigned8, layout->dimType(riegl_visible));
    EXPECT_EQ(Dimension::Type::Unsigned8, layout->dimType(riegl_selected));
    EXPECT_EQ(Dimension::Type::Unsigned8, layout->dimType(riegl_end_of_line));

    checkDimension(view,  0, riegl_visible,     uint8_t(1), 0);
    checkDimension(view,  0, riegl_selected,    uint8_t(0), 0);
    checkDimension(view,  0, riegl_end_of_line, uint8_t(0), 0);

    checkDimension(view, 99, riegl_visible,     uint8_t(1), 0);
    checkDimension(view, 99, riegl_selected,    uint8_t(0), 0);
    checkDimension(view, 99, riegl_end_of_line, uint8_t(0), 0);
}
