/******************************************************************************
* Copyright (c) 2012, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <boost/uuid/uuid_io.hpp>

#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/StageFactory.hpp>
#include <las/LasWriter.hpp>

#include "Support.hpp"

#include <iostream>

#ifdef PDAL_COMPILER_GCC
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif


using namespace pdal;

TEST(NitfReaderTest, test_one)
{
    StageFactory f;

    Options nitf_opts;
    nitf_opts.add("filename", Support::datapath("nitf/autzen-utm10.ntf"));
    nitf_opts.add("count", 750);

    PointTable table;

    std::shared_ptr<Stage> nitf_reader(f.createStage("readers.nitf"));
    EXPECT_TRUE(nitf_reader.get());
    nitf_reader->setOptions(nitf_opts);
    nitf_reader->prepare(table);
    PointViewSet pbSet = nitf_reader->execute(table);
    EXPECT_EQ(pbSet.size(), 1u);
    PointViewPtr view = *pbSet.begin();

    // check metadata
//ABELL
/**
    {
        Metadata metadata = nitf_reader.getMetadata();
        /////////////////////////////////////////////////EXPECT_EQ(metadatums.size(), 80u);
        EXPECT_EQ(metadata.toPTree().get<std::string>("metadata.FH_FDT.value"), "20120323002946");
    }
**/

    //
    // read LAS
    //
    Options las_opts;
    las_opts.add("count", 750);
    las_opts.add("filename", Support::datapath("nitf/autzen-utm10.las"));

    PointTable table2;

    std::shared_ptr<Stage> las_reader(f.createStage("readers.las"));
    EXPECT_TRUE(las_reader.get());
    las_reader->setOptions(las_opts);
    las_reader->prepare(table2);
    PointViewSet pbSet2 = las_reader->execute(table2);
    EXPECT_EQ(pbSet2.size(), 1u);
    PointViewPtr view2 = *pbSet.begin();
    //
    //
    // compare the two views
    //
    EXPECT_EQ(view->size(), view2->size());

    for (PointId i = 0; i < view2->size(); i++)
    {
        int32_t nitf_x = view->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t nitf_y = view->getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t nitf_z = view->getFieldAs<int32_t>(Dimension::Id::Z, i);

        int32_t las_x = view2->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t las_y = view2->getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t las_z = view2->getFieldAs<int32_t>(Dimension::Id::Z, i);

        EXPECT_EQ(nitf_x, las_x);
        EXPECT_EQ(nitf_y, las_y);
        EXPECT_EQ(nitf_z, las_z);
    }
}


TEST(NitfReaderTest, test_chipper)
{
    Option option("filename", Support::configuredpath("nitf/chipper.xml"));
    Options options(option);

    PointTable table;

    PipelineManager mgr;
    PipelineReader specReader(mgr);
    specReader.readPipeline(Support::configuredpath("nitf/chipper.xml"));
    //ABELL - need faux writer or something.
    /**
    mgr.execute();
    StageSequentialIterator* iter = reader.createSequentialIterator(data);
    const uint32_t num_read = iter->read(data);
    EXPECT_EQ(num_read, 13u);

    uint32_t num_blocks = chipper->GetBlockCount();
    EXPECT_EQ(num_blocks, 8u);
    **/
}

TEST(NitfReaderTest, optionSrs)
{
    StageFactory f;

    Options nitfOpts;
    nitfOpts.add("filename", Support::datapath("nitf/autzen-utm10.ntf"));

    std::string sr = "PROJCS[\"NAD83 / UTM zone 11N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.257222101,AUTHORITY[\"EPSG\",\"7019\"]],TOWGS84[0,0,0,0,0,0,0],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-123],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH],AUTHORITY[\"EPSG\",\"26910\"]]";

    nitfOpts.add("spatialreference", sr);

    PointTable table;

    std::shared_ptr<Stage> nitfReader(f.createStage("readers.nitf"));
    EXPECT_TRUE(nitfReader.get());
    nitfReader->setOptions(nitfOpts);

    Options lasOpts;
    lasOpts.add("filename", "/dev/null");

    LasWriter writer;
    writer.setInput(*nitfReader);
    writer.setOptions(lasOpts);;

    writer.prepare(table);
    PointViewSet pbSet = writer.execute(table);

    EXPECT_EQ(sr, nitfReader->getSpatialReference().getWKT());
    EXPECT_EQ("", writer.getSpatialReference().getWKT());
    EXPECT_EQ(sr, table.spatialRef().getWKT());
}
