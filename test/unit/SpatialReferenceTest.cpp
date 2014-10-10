/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <boost/test/unit_test.hpp>

#include <pdal/SpatialReference.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(SpatialReferenceTest)

using namespace pdal;

#ifdef PDAL_SRS_ENABLED

BOOST_AUTO_TEST_CASE(test_env_vars)
{

#ifdef _MSC_VER
    const char* gdal_data = getenv("GDAL_DATA");
    const char* proj_lib = getenv("PROJ_LIB");

    BOOST_CHECK(pdal::FileUtils::fileExists(gdal_data));
    BOOST_CHECK(pdal::FileUtils::fileExists(proj_lib));
#endif
}


BOOST_AUTO_TEST_CASE(test_ctor)
{
    pdal::SpatialReference srs;

    BOOST_CHECK(srs.getProj4() == "");
    BOOST_CHECK(srs.getWKT() == "");
    BOOST_CHECK(srs.empty());
}


// Test round-tripping proj.4 string
BOOST_AUTO_TEST_CASE(test_proj4_roundtrip)
{
    using namespace pdal;

    std::string proj4 = "+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs";
    std::string proj4_ellps =
        "+proj=utm +zone=15 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    std::string proj4_out =
        "+proj=utm +zone=15 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m "
        "+no_defs";

    {
        SpatialReference ref;
        ref.setProj4(proj4);
        BOOST_CHECK(!ref.empty());
        const std::string ret = ref.getProj4();
        BOOST_CHECK(ret == proj4_out);
    }

    {
        SpatialReference ref;
        ref.setProj4(proj4_ellps);
        const std::string ret = ref.getProj4();
        //BOOST_CHECK(ret == proj4);
        BOOST_CHECK(ret == proj4_out);
    }

    {
        SpatialReference ref;
        ref.setProj4(proj4_out);
        const std::string ret = ref.getProj4();
        BOOST_CHECK(ret == proj4_out);
    }
}


// Test setting EPSG:4326 from User string
BOOST_AUTO_TEST_CASE(test_userstring_roundtrip)
{
    pdal::SpatialReference ref;

    std::string code = "EPSG:4326";
    std::string proj4 = "+proj=longlat +datum=WGS84 +no_defs";
    std::string proj4_ellps =
        "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
    const std::string wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
    ref.setFromUserInput(code);

    std::string ret_proj = ref.getProj4();
    std::string ret_wkt = ref.getWKT();

    BOOST_CHECK(ret_proj == proj4);
    BOOST_CHECK(ret_wkt == wkt);
}


// Test fetching SRS from an existing file
BOOST_AUTO_TEST_CASE(test_read_srs)
{
    using namespace pdal;
    using namespace pdal::drivers;

    PointContext ctx;

    Options ops;
    ops.add("filename", Support::datapath("las/utm17.las"));
    las::Reader reader;
    reader.setOptions(ops);
    reader.prepare(ctx);
    reader.execute(ctx);

    const pdal::SpatialReference& ref = reader.getSpatialReference();

    const std::string ret_wkt = ref.getWKT();
    const std::string ret_proj4 = ref.getProj4();

    const std::string wkt = "PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-81],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"32617\"]]";

    BOOST_CHECK(ret_wkt == wkt);

    std::string proj4 = "+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs";
    BOOST_CHECK(ret_proj4 == proj4);
}


// Try writing a compound coordinate system to file and ensure we get back
// WKT with the geoidgrids (from the WKT VLR).
BOOST_AUTO_TEST_CASE(test_vertical_datums)
{
    using namespace pdal;
    using namespace pdal::drivers;

    std::string tmpfile(Support::temppath("tmp_srs.las"));
    FileUtils::deleteFile(tmpfile);

    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

    {
        SpatialReference ref;
        ref.setFromUserInput(wkt);
        {
            const std::string wkt2 = ref.getWKT(SpatialReference::eCompoundOK);
            BOOST_CHECK(wkt == wkt2); // just to make sure
        }

        PointContext ctx;
        // Write a very simple file with our SRS and one point.
        Options ops1;
        ops1.add("filename", Support::datapath("las/1.2-with-color.las"));
        las::Reader reader;
        reader.setOptions(ops1);

        const uint64_t numPoints = reader.getNumPoints();

        // need to scope the writer, so that's it dtor can use the stream
        Options opts;

        opts.add("filename", tmpfile);

        las::Writer writer;
        writer.setOptions(opts);
        writer.setInput(&reader);
        writer.prepare(ctx);
        writer.setSpatialReference(ref);
        writer.execute(ctx);
    }

    // Reopen and check contents.
    {
        PointContext ctx;
        Options ops;
        ops.add("filename", tmpfile);
        las::Reader reader;
        reader.setOptions(ops);
        reader.prepare(ctx);
        reader.execute(ctx);

        const SpatialReference ref2 = reader.getSpatialReference();
        const std::string wkt2 =
            ref2.getWKT(pdal::SpatialReference::eCompoundOK);

        BOOST_CHECK(wkt == wkt2);
    }

    // Cleanup
    FileUtils::deleteFile(tmpfile);
}


// Try writing only the WKT VLR to a file, and see if the resulting
// file still works ok.
BOOST_AUTO_TEST_CASE(test_writing_vlr)
{
    using namespace pdal;
    using namespace pdal::drivers;

    std::string tmpfile(Support::temppath("tmp_srs_9.las"));
    SpatialReference ref;

    const std::string reference_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";

    ref.setFromUserInput("EPSG:4326");
    std::string wkt = ref.getWKT();
    BOOST_CHECK(wkt == reference_wkt);

    // Write a very simple file with our SRS and one point.
    {
        pdal::FileUtils::deleteFile(tmpfile);

        PointContext ctx;
        pdal::drivers::las::Reader readerx;
        Options readerOpts;

        readerOpts.add("filename",
            ::Support::datapath("las/1.2-with-color.las"));
        readerx.setOptions(readerOpts);

        Options writerOpts;
        las::Writer writer;

        writerOpts.add("filename", tmpfile);
        writer.setOptions(writerOpts);
        writer.setInput(&readerx);
        writer.prepare(ctx);
        writer.setSpatialReference(ref);
        writer.execute(ctx);
    }

    // Reopen and check contents.
    {
        PointContext ctx;
        Options ops;
        ops.add("filename", tmpfile);
        las::Reader reader;
        reader.setOptions(ops);
        reader.prepare(ctx);
        reader.execute(ctx);

        SpatialReference result_ref = reader.getSpatialReference();

        BOOST_CHECK_EQUAL(reader.header().vlrCount(), 4);
        std::string wkt = result_ref.getWKT();
        BOOST_CHECK_EQUAL(wkt, reference_wkt);
    }

    // Cleanup
    FileUtils::deleteFile(tmpfile);
}



BOOST_AUTO_TEST_CASE(test_io)
{
    using namespace pdal;

    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

    SpatialReference ref;
    ref.setFromUserInput(wkt);

    std::stringstream ss(std::stringstream::in | std::stringstream::out);

    ss << ref;

    SpatialReference ref2;
    ss >> ref2;

    BOOST_CHECK(ref == ref2);
}

BOOST_AUTO_TEST_CASE(test_vertical_and_horizontal)
{

    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";
    pdal::SpatialReference srs(wkt);

    std::string horiz = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
    std::string vert = "VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]";
    std::string horizontal = srs.getHorizontal();
    std::string vertical = srs.getVertical();

    BOOST_CHECK_EQUAL(horiz, horizontal);
    BOOST_CHECK_EQUAL(vert, vertical);

}

#endif

BOOST_AUTO_TEST_SUITE_END()
