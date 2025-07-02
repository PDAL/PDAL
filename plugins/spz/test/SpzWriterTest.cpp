
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

#include "SpzWriter.hpp"
#include "SpzReader.hpp"

using namespace pdal;

PointViewPtr setXYZ(PointTableRef table)
{
    table.layout()->registerDim(Dimension::Id::X);
    table.layout()->registerDim(Dimension::Id::Y);
    table.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v(new PointView(table));
    v->setField(Dimension::Id::X, 0, 1);
    v->setField(Dimension::Id::Y, 0, 1);
    v->setField(Dimension::Id::Z, 0, 0);

    v->setField(Dimension::Id::X, 1, 2);
    v->setField(Dimension::Id::Y, 1, 1);
    v->setField(Dimension::Id::Z, 1, 0);

    v->setField(Dimension::Id::X, 2, 1);
    v->setField(Dimension::Id::Y, 2, 2);
    v->setField(Dimension::Id::Z, 2, 0);

    return v;
}

TEST(SpzWriterTest, xyz_only_test)
{
    BufferReader r;
    PointTable t;
    PointViewPtr v = setXYZ(t);
    r.addView(v);

    SpzWriter writer;
    Options opts;
    std::string path = Support::temppath("out.spz");
    opts.replace("filename", path);
    writer.setInput(r);
    writer.setOptions(opts);

    writer.prepare(t);
    writer.execute(t);
    ASSERT_EQ(FileUtils::fileSize(path), 53);

    SpzReader reader;
    // using same options, just the filename
    reader.setOptions(opts);
    
    PointTable readTable;
    reader.prepare(readTable);
    PointViewSet viewSet = reader.execute(readTable);
    PointViewPtr readView = *viewSet.begin();
    EXPECT_EQ(readView->size(), 3);

    // Checking X values. These are packed/unpacked with more precision
    //than the other fields.
    EXPECT_FLOAT_EQ(readView->getFieldAs<float>(Dimension::Id::X, 0), 1.0);
    EXPECT_FLOAT_EQ(readView->getFieldAs<float>(Dimension::Id::X, 1), 2.0);
    EXPECT_FLOAT_EQ(readView->getFieldAs<float>(Dimension::Id::X, 2), 1.0);
}

TEST(SpzWriterTest, all_dimensions_test)
{
    PointTable table;
    BufferReader r;

    // registering dims
    Dimension::Id alphaId = table.layout()->assignDim("opacity", Dimension::Type::Float);

    Dimension::IdList scaleIds;
    Dimension::IdList colorIds;
    Dimension::IdList shIds;
    Dimension::IdList rotIds;
    for (int i = 0; i < 3; ++i)
    {
        scaleIds.push_back(table.layout()->assignDim("scale_" + std::to_string(i),
            Dimension::Type::Float));
        colorIds.push_back(table.layout()->assignDim("f_dc_" + std::to_string(i),
            Dimension::Type::Float));
    }
    for (int i = 0; i < 4; ++i)
        rotIds.push_back(table.layout()->assignDim("rot_" + std::to_string(i),
            Dimension::Type::Float));
    for (int i = 0; i < 9; ++i)
        shIds.push_back(table.layout()->assignDim("f_rest_" + std::to_string(i),
            Dimension::Type::Float));
        
    PointViewPtr v = setXYZ(table);
    
    // setting dims - set rotation later
    // using the same values for each dimension (except rotation)
    std::array<float, 3> values{0.2, 0.4, 0.6};
    for (int i = 0; i < 3; i++)
    {
        v->setField(alphaId, i, values[i]);
        for (auto c : colorIds)
            v->setField(c, i, values[i]);
        for (auto s : scaleIds)
            v->setField(s, i, values[i]);
        for (auto sh : shIds)
            v->setField(sh, i, values[i]);
    }
        
    // Set rotation dimension:
    // make a valid quaternion
    std::array<float, 4> in{0.1, 0.5, 0.3, 0.2};
    float norm = std::sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2] + in[3] * in[3]);
    std::array<float, 4> quat{in[0] / norm, in[1] / norm, in[2] / norm, in[3] / norm};
    // Set rotation
    for (auto i = 0; i < 4; i++)
       v->setField(rotIds[i], 0, quat[i]);

    r.addView(v);

    SpzWriter writer;
    Options opts;
    std::string path = Support::temppath("out.spz");
    opts.replace("filename", path);
    writer.setInput(r);
    writer.setOptions(opts);

    writer.prepare(table);
    writer.execute(table);
    ASSERT_EQ(FileUtils::fileSize(path), 82);

    SpzReader reader;
    // using same options, just the filename
    reader.setOptions(opts);
    
    PointTable readTable;
    reader.prepare(readTable);
    PointViewSet viewSet = reader.execute(readTable);
    PointViewPtr readView = *viewSet.begin();
    EXPECT_EQ(readView->size(), 3);

    // Values in the resulting spz are very off. It's on
    //their end AFAIK (the packing/unpacking is lossy)
    float tolerance = 0.03;
    // alphas are packed/unpacked more accurately.
    float alphaTolerance = 0.01;
    for (PointId i = 0; i < readView->size(); ++i)
    {
        EXPECT_NEAR(readView->getFieldAs<float>(alphaId, i), values[i], alphaTolerance);
        for (auto s : scaleIds)
            EXPECT_NEAR(readView->getFieldAs<float>(s, i), values[i], tolerance);
        for (auto c : colorIds)
            EXPECT_NEAR(readView->getFieldAs<float>(c, i), values[i], tolerance);
        for (auto sh : shIds)
            EXPECT_NEAR(readView->getFieldAs<float>(sh, i), values[i], tolerance);
    }

    // check rotation (first point only)
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(readView->getFieldAs<float>(rotIds[i], 0), quat[i], tolerance);
}
