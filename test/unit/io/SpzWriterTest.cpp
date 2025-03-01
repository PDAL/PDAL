
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

#include <io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/util/FileUtils.hpp>

#include <io/SpzWriter.hpp>

#include <spz/src/cc/load-spz.h>

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

PointViewPtr setAllDims(PointTableRef table)
{
    // registering dims
    Dimension::Id alphaId = table.layout()->assignDim("opacity", Dimension::Type::Float);

    Dimension::IdList scaleIds;
    Dimension::IdList colorIds;
    for (int i = 0; i < 3; ++i)
    {
        scaleIds.push_back(table.layout()->assignDim("scale_" + std::to_string(i),
            Dimension::Type::Float));
        colorIds.push_back(table.layout()->assignDim("f_dc_" + std::to_string(i),
            Dimension::Type::Float));
    }
    Dimension::IdList rotIds;
    for (int i = 0; i < 4; ++i)
        rotIds.push_back(table.layout()->assignDim("rot_" + std::to_string(i),
            Dimension::Type::Float));
    Dimension::IdList shIds;
    for (int i = 0; i < 9; ++i)
        shIds.push_back(table.layout()->assignDim("f_rest_" + std::to_string(i),
            Dimension::Type::Float));
        
    PointViewPtr v = setXYZ(table);
    
    // setting dims - set rotation later
    for (int i = 0; i < 3; i++)
    {
        v->setField(alphaId, i, (0.05f * (i + 1)));
        for (auto c : colorIds)
            v->setField(c, i, (0.05f * (i + 1)));
        for (auto s : scaleIds)
            v->setField(s, i, (0.1f * (i + 1)));
        for (auto sh : shIds)
            v->setField(sh, i, (0.3f * (i + 1)));
    }

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

    // check values: could use SPZ reader too, but don't want to rely on
    //it being correct
    spz::GaussianCloud g = spz::loadSpz(path);
    EXPECT_EQ(g.numPoints, 3);
    // Checking X values. These are packed/unpacked with more precision
    //than the other fields.
    EXPECT_FLOAT_EQ(g.positions[0], 1.0);
    EXPECT_FLOAT_EQ(g.positions[3], 2.0);
    EXPECT_FLOAT_EQ(g.positions[6], 1.0);
}

TEST(SpzWriterTest, all_dimensions_test)
{
    PointTable t;
    BufferReader r;
    PointViewPtr v = setAllDims(t);
        
    // Set rotation dimension
    // make a valid quaternion
    std::array<float, 4> in{0.1, 0.5, 0.3, 0.2};
    float norm = std::sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2] + in[3] * in[3]);
    std::array<float, 4> quat{in[0] / norm, in[1] / norm, in[2] / norm, in[3] / norm};

    // Set rotation
    v->setField(v->layout()->findProprietaryDim("rot_0"), 0, quat[0]);
    v->setField(v->layout()->findProprietaryDim("rot_1"), 0, quat[1]);
    v->setField(v->layout()->findProprietaryDim("rot_2"), 0, quat[2]);
    v->setField(v->layout()->findProprietaryDim("rot_3"), 0, quat[3]);

    r.addView(v);

    SpzWriter writer;
    Options opts;
    std::string path = Support::temppath("out.spz");
    opts.replace("filename", path);
    writer.setInput(r);
    writer.setOptions(opts);

    writer.prepare(t);
    writer.execute(t);
    ASSERT_EQ(FileUtils::fileSize(path), 82);

    spz::GaussianCloud g = spz::loadSpz(path);
    EXPECT_EQ(g.numPoints, 3);
    // checking values
    EXPECT_EQ(g.shDegree, 1);

    // Values in the GaussianCloud are very off. It's on
    //their end AFAIK (the packing/unpacking is lossy)
    float tolerance = 0.03;
    EXPECT_NEAR(g.colors[0], 0.05, tolerance);
    EXPECT_NEAR(g.colors[3], 0.1, tolerance);
    EXPECT_NEAR(g.colors[6], 0.15, tolerance);

    // alphas are packed/unpacked more accurately.
    float alphaTolerance = 0.005;
    EXPECT_NEAR(g.alphas[0], 0.05, alphaTolerance);
    EXPECT_NEAR(g.alphas[1], 0.1, alphaTolerance);
    EXPECT_NEAR(g.alphas[2], 0.15, alphaTolerance);

    EXPECT_NEAR(g.scales[0], 0.1, tolerance);
    EXPECT_NEAR(g.scales[3], 0.2, tolerance);
    EXPECT_NEAR(g.scales[6], 0.3, tolerance);

    EXPECT_NEAR(g.sh[0], 0.3, tolerance);
    EXPECT_NEAR(g.sh[9], 0.6, tolerance);
    EXPECT_NEAR(g.sh[18], 0.9, tolerance); 

    // check rotation (first point only): wxyz -> xyzw
    EXPECT_NEAR(g.rotations[0], quat[1], tolerance);
    EXPECT_NEAR(g.rotations[1], quat[2], tolerance);
    EXPECT_NEAR(g.rotations[2], quat[3], tolerance);
    EXPECT_NEAR(g.rotations[3], quat[0], tolerance);
}
