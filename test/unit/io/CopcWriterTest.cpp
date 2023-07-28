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

#include <io/CopcReader.hpp>
#include <io/CopcWriter.hpp>
#include <io/LasReader.hpp>

#include <pdal/PDALUtils.hpp>

#include "Support.hpp"

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
        options.add("srs_consume_preference", "projjson, wkt2");

        CopcReader creader;
        creader.setOptions(options);

        const QuickInfo qi(creader.preview());
        std::string srs = qi.m_srs.getPROJJSON();

        EXPECT_TRUE(Utils::startsWith(srs, "{\n  \"type\": \"DerivedProjectedCRS\","));
    }
}

} // namespace pdal
