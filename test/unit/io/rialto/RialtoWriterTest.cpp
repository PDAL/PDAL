#include "gtest/gtest.h"

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include "RialtoWriter.hpp"
#include "Support.hpp"

using namespace pdal;

TEST(RialtoWriterTest, createWriter)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    EXPECT_TRUE(writer.get());
}

TEST(RialtoWriterTest, testConstructor)
{
    std::unique_ptr<RialtoWriter> writer(new RialtoWriter);
    EXPECT_EQ(writer->getName(), "writers.rialto");
}

TEST(RialtoWriterTest, testWriteHeaderOverwrite)
{
    FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    BOX3D bounds(1.0, 2.0, 3.0, 11.0, 12.0, 13.0);

    Options ro;
    ro.add("bounds", bounds);
    ro.add("count", 10);
    ro.add("mode", "ramp");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ro);

    Options wo;
    wo.add("filename", Support::temppath("RialtoTest"));
    wo.add("max_level", 0);
    wo.add("overwrite", true);

    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    writer->setOptions(wo);
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);
    writer->execute(table);

    bool ok = Support::compare_text_files(Support::temppath("RialtoTest/header.json"),
                                          Support::datapath("io/header.json"));

    if (ok)
        FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    EXPECT_TRUE(ok);
}

TEST(RialtoWriterTest, testWriteHeaderNoOverwrite)
{
    FileUtils::deleteFile(Support::temppath("RialtoTest/header.json"));

    BOX3D bounds(1.0, 2.0, 3.0, 11.0, 12.0, 13.0);

    Options ro;
    ro.add("bounds", bounds);
    ro.add("count", 10);
    ro.add("mode", "ramp");

    StageFactory f;
    std::unique_ptr<Stage> reader(f.createStage("readers.faux"));
    reader->setOptions(ro);

    Options wo;
    wo.add("filename", Support::temppath("RialtoTest"));
    wo.add("max_level", 0);
    wo.add("overwrite", false);

    std::unique_ptr<Stage> writer(f.createStage("writers.rialto"));
    writer->setOptions(wo);
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);
    EXPECT_THROW(writer->execute(table), pdal_error);
}

