/******************************************************************************
 * Copyright (c) 2021, Hobu Inc. (info@hobu.co)
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

#include <algorithm>

#include <pdal/pdal_test_main.hpp>

#include <pdal/util/FileUtils.hpp>
#include <io/BufferReader.hpp>
#include <io/CopcReader.hpp>
#include <io/CopcWriter.hpp>
#include <io/LasReader.hpp>
#include <filters/FerryFilter.hpp>

#include <pdal/PDALUtils.hpp>

#include "Support.hpp"

#include <gdal_version.h>

namespace pdal
{

namespace
{
    std::string wkt2DerivedProjected =
        "DERIVEDPROJCRS[\"Custom Site Calibrated CRS\",\n"
        "    BASEPROJCRS[\"NAD83(2011) / Mississippi East (ftUS)\",\n"
        "        BASEGEOGCRS[\"NAD83(2011)\",\n"
        "            DATUM[\"NAD83 (National Spatial Reference System "
        "2011)\",\n"
        "                ELLIPSOID[\"GRS 1980\",6378137,298.257222101,\n"
        "                    LENGTHUNIT[\"metre\",1]]],\n"
        "            PRIMEM[\"Greenwich\",0,\n"
        "                ANGLEUNIT[\"degree\",0.0174532925199433]]],\n"
        "        CONVERSION[\"SPCS83 Mississippi East zone (US Survey "
        "feet)\",\n"
        "            METHOD[\"Transverse Mercator\",\n"
        "                ID[\"EPSG\",9807]],\n"
        "            PARAMETER[\"Latitude of natural origin\",29.5,\n"
        "                ANGLEUNIT[\"degree\",0.0174532925199433],\n"
        "                ID[\"EPSG\",8801]],\n"
        "            PARAMETER[\"Longitude of natural "
        "origin\",-88.8333333333333,\n"
        "                ANGLEUNIT[\"degree\",0.0174532925199433],\n"
        "                ID[\"EPSG\",8802]],\n"
        "            PARAMETER[\"Scale factor at natural origin\",0.99995,\n"
        "                SCALEUNIT[\"unity\",1],\n"
        "                ID[\"EPSG\",8805]],\n"
        "            PARAMETER[\"False easting\",984250,\n"
        "                LENGTHUNIT[\"US survey foot\",0.304800609601219],\n"
        "                ID[\"EPSG\",8806]],\n"
        "            PARAMETER[\"False northing\",0,\n"
        "                LENGTHUNIT[\"US survey foot\",0.304800609601219],\n"
        "                ID[\"EPSG\",8807]]]],\n"
        "    DERIVINGCONVERSION[\"Affine transformation as PROJ-based\",\n"
        "        METHOD[\"PROJ-based operation method: "
        "+proj=pipeline +step +proj=unitconvert +xy_in=m +xy_out=us-ft "
        "+step +proj=affine +xoff=20 "
        "+step +proj=unitconvert +xy_in=us-ft +xy_out=m\"]],\n"
        "    CS[Cartesian,2],\n"
        "        AXIS[\"northing (Y)\",north,\n"
        "            LENGTHUNIT[\"US survey foot\",0.304800609601219]],\n"
        "        AXIS[\"easting (X)\",east,\n"
        "            LENGTHUNIT[\"US survey foot\",0.304800609601219]],\n"
        "    REMARK[\"EPSG:6507 with 20 feet offset and axis inversion\"]]";
}

TEST(CopcWriterTest, srsWkt2)
{
#if GDAL_VERSION_NUM <= GDAL_COMPUTE_VERSION(3,6,0)
    // not working with PROJ >= 9.2.0 https://github.com/OSGeo/gdal/pull/6800
    std::cerr << "Test disabled with GDAL <= 3.6.0" << std::endl;
    return;
#endif
    const auto filename = Support::temppath("srsWkt2.copc.las");
    {
        Options readerOps;
        readerOps.add("filename", Support::datapath("las/utm15.las"));

        LasReader reader;
        reader.setOptions(readerOps);

        Options writerOps;
        writerOps.add("filename", filename);
        writerOps.add("a_srs", wkt2DerivedProjected);
        writerOps.add("enhanced_srs_vlrs", true);
        CopcWriter writer;
        writer.setInput(reader);
        writer.setOptions(writerOps);

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    {
        Options options;
        options.add("filename", filename);

        CopcReader creader;
        creader.setOptions(options);

        const QuickInfo qi(creader.preview());
        std::string srs = qi.m_srs.getWKT();

        EXPECT_TRUE(Utils::startsWith(srs, "DERIVEDPROJCRS[\"Custom Site Calibrated CRS\""));
    }

    {
        Options options;
        options.add("filename", filename);
        options.add("srs_vlr_order", "projjson, wkt2");

        CopcReader creader;
        creader.setOptions(options);

        const QuickInfo qi(creader.preview());
        std::string srs = qi.m_srs.getPROJJSON();

        EXPECT_TRUE(Utils::startsWith(srs, "{\n  \"type\": \"DerivedProjectedCRS\","));
    }
}

TEST(CopcWriterTest, srsUTM)
{
    const auto filename = Support::temppath("srs.copc.las");
    {
        Options readerOps;
        readerOps.add("filename", Support::datapath("las/utm15.las"));

        LasReader reader;
        reader.setOptions(readerOps);

        Options writerOps;
        writerOps.add("filename", filename);
        writerOps.add("enhanced_srs_vlrs", true);
        CopcWriter writer;
        writer.setInput(reader);
        writer.setOptions(writerOps);

        PointTable table;
        writer.prepare(table);
        writer.execute(table);
    }

    Options ops;
    ops.add("filename", filename);

    LasReader r;
    r.setOptions(ops);

    PointTable t;
    r.prepare(t);
    r.execute(t);

    const QuickInfo qi(r.preview());
    std::string srs = qi.m_srs.getWKT();
    EXPECT_TRUE(Utils::startsWith(srs, "PROJCRS[\"NAD83 / UTM zone 15N\",BASEGEOGCRS"));

    const char *data = nullptr;

    EXPECT_TRUE(r.vlrData("LASF_Projection", 4224, data) > 0);
    EXPECT_TRUE(Utils::startsWith(data, "PROJCRS[\"NAD83 / UTM zone 15N\""));

    data = nullptr;
    EXPECT_TRUE(r.vlrData("PDAL", 4225, data) > 0);
    EXPECT_TRUE(Utils::startsWith(data, "{\n  \"type\": \"ProjectedCRS\","));

    data = nullptr;
    // This vlr data must not be null terminated and segfaults when startsWith
    // tries to read it
    EXPECT_TRUE(r.vlrData("LASF_Projection", 2112, data) > 0);
    std::string info (data, 50);
    bool test = Utils::startsWith(info, "PROJCS[\"NAD83 / UTM zone 15N\"" );
    EXPECT_TRUE(test);
}

TEST(CopcWriterTest, scaling)
{
    using namespace Dimension;

    const std::string filename(Support::temppath("copc_scaling.las"));
    PointTable table;

    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 1406018.497);
    view->setField(Id::Y, 0, 4917487.174);
    view->setField(Id::Z, 0, 62.276);
    bufferReader.addView(view);

    Options writerOps;
    writerOps.add("filename", filename);
    writerOps.add("offset_x", "1000000");
    writerOps.add("scale_x", "0.001");
    writerOps.add("offset_y", "5000000");
    writerOps.add("scale_y", "0.001");
    writerOps.add("offset_z", "0");
    writerOps.add("scale_z", "0.001");

    CopcWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(bufferReader);

    writer.prepare(table);
    writer.execute(table);

    Options readerOps;
    readerOps.add("filename", filename);

    PointTable readTable;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(readTable);
    PointViewSet viewSet = reader.execute(readTable);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1u);
    EXPECT_NEAR(1406018.497, view->getFieldAs<double>(Id::X, 0), .00001);
    EXPECT_NEAR(4917487.174, view->getFieldAs<double>(Id::Y, 0), .00001);
    EXPECT_NEAR(62.276, view->getFieldAs<double>(Id::Z, 0), .00001);
    FileUtils::deleteFile(filename);
}

TEST(CopcWriterTest, extradim)
{
    std::string filename(Support::datapath("las/1.2-with-color.las"));
    std::string outFilename(Support::temppath("copcdims.copc.laz"));

    FileUtils::deleteFile(outFilename);

    auto createFile = [&](const std::string& extraDims)
    {

        LasReader r;

        Options ro;
        ro.add("filename", filename);

        r.setOptions(ro);

        // Make Q, R & S as extra dims.
        FerryFilter f;
        Options fo;
        fo.add("dimensions", "X=>Q, Y=>R, Red=>S");

        f.setOptions(fo);
        f.setInput(r);

        CopcWriter w;
        Options wo;
        wo.add("filename", outFilename);
        wo.add("extra_dims", extraDims);

        w.setOptions(wo);
        w.setInput(f);

        PointTable t;
        w.prepare(t);
        w.execute(t);
    };

    auto verifyFile = [&](bool q, bool r, bool s) -> bool
    {
        LasReader r2;
        Options r2o;
        r2o.add("filename", outFilename);

        r2.setOptions(r2o);

        PointTable t2;
        r2.prepare(t2);
        PointLayoutPtr layout = t2.layout();

        FileUtils::deleteFile(outFilename);
        return ((q == (layout->findDim("Q") != Dimension::Id::Unknown)) &&
                (r == (layout->findDim("R") != Dimension::Id::Unknown)) &&
                (s == (layout->findDim("S") != Dimension::Id::Unknown)));
    };


    createFile("Q=int32");                                // Q, but no R, S
    EXPECT_TRUE(verifyFile(true, false, false));
    createFile("all");                                    // Q, R, and S
    EXPECT_TRUE(verifyFile(true, true, true));
    createFile("Q=int32, S=double");                      // Q, and S, no R
    EXPECT_TRUE(verifyFile(true, false, true));

    EXPECT_THROW(createFile("Q=int32, S"), pdal_error);   // No type for S
    EXPECT_THROW(createFile("X=int32"), pdal_error);      // Existing dimension.
    EXPECT_THROW(createFile("Z=int32"), pdal_error);      // Unknown dimension.
}

} // namespace pdal
