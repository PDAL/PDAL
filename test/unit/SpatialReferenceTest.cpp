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
#include <pdal/Utils.hpp>
#include <pdal/drivers/las/VariableLengthRecord.hpp>
#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(SpatialReferenceTest)


BOOST_AUTO_TEST_CASE(test_env_vars)
{
    
#ifdef _MSC_VER
    const char* gdal_data = getenv("GDAL_DATA");
    const char* proj_lib = getenv("PROJ_LIB");

    BOOST_CHECK(pdal::Utils::fileExists(gdal_data));
    BOOST_CHECK(pdal::Utils::fileExists(proj_lib));
#endif
    return;
}


BOOST_AUTO_TEST_CASE(test_ctor)
{
    pdal::SpatialReference srs;

    BOOST_CHECK(srs.getProj4() == "");
    BOOST_CHECK(srs.getWKT() == "");

    return;
}


// Test round-tripping proj.4 string
BOOST_AUTO_TEST_CASE(test_proj4_roundtrip)
{
    const std::string proj4 = "+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs";
    const std::string proj4_ellps = "+proj=utm +zone=15 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
    const std::string proj4_out = "+proj=utm +zone=15 +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs";

    {
        pdal::SpatialReference ref;
        ref.setProj4(proj4);
        const std::string ret = ref.getProj4();
        //BOOST_CHECK(ret == proj4);
        BOOST_CHECK(ret == proj4_out);
    }

    {
        pdal::SpatialReference ref;
        ref.setProj4(proj4_ellps);
        const std::string ret = ref.getProj4();
        //BOOST_CHECK(ret == proj4);
        BOOST_CHECK(ret == proj4_out);
    }

    {
        pdal::SpatialReference ref;
        ref.setProj4(proj4_out);
        const std::string ret = ref.getProj4();
        BOOST_CHECK(ret == proj4_out);
    }

    return;
}


// Test setting EPSG:4326 from User string
BOOST_AUTO_TEST_CASE(test_userstring_roundtrip)
{
    pdal::SpatialReference ref;

    const std::string code = "EPSG:4326";
    const std::string proj4 = "+proj=longlat +datum=WGS84 +no_defs";
    const std::string proj4_ellps = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
    const std::string wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
    ref.setFromUserInput(code);

    const std::string ret_proj = ref.getProj4();
    const std::string ret_wkt = ref.getWKT();

    BOOST_CHECK(ret_proj == proj4);
    BOOST_CHECK(ret_wkt == wkt);

    return;
}

       
// Test fetching SRS from an existing file
BOOST_AUTO_TEST_CASE(test_read_srs)
{
    pdal::drivers::las::LasReader reader(Support::datapath("utm17.las"));

    const pdal::SpatialReference& ref = reader.getSpatialReference();

    BOOST_CHECK(reader.getVLRs().size() == 3);

    const std::string ret_wkt = ref.getWKT();
    const std::string ret_proj4 = ref.getProj4();

    const std::string wkt = "PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-81],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"32617\"]]";
    BOOST_CHECK(ret_wkt == wkt);

    const std::string proj4 = "+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs";
    BOOST_CHECK(ret_proj4 == proj4);

    return;
}
    

// Test VLR sizes from setting SRS
BOOST_AUTO_TEST_CASE(test_vlr_sizes)
{
    pdal::SpatialReference ref;
    const char* code = "EPSG:4326";
    ref.setFromUserInput(code);

    std::vector<pdal::drivers::las::VariableLengthRecord> vlrs;
    pdal::drivers::las::VariableLengthRecord::setVLRsFromSRS(ref, vlrs, pdal::SpatialReference::eCompoundOK);

    BOOST_CHECK(vlrs.size() == boost::uint32_t(4));
    BOOST_CHECK(vlrs[0].getLength() == boost::uint32_t(64));

    return;
}


// Test incorporation of vertical datum information into WKT string and
// into GeoTIFF VLRs. 
BOOST_AUTO_TEST_CASE(test_vertical_datum)
{
    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

    pdal::SpatialReference ref;
    {
        ref.setFromUserInput(wkt);
        BOOST_CHECK(ref.getWKT(pdal::SpatialReference::eCompoundOK) == wkt);
    }

    std::vector<pdal::drivers::las::VariableLengthRecord> vlrs;
    {
        pdal::drivers::las::VariableLengthRecord::setVLRsFromSRS(ref, vlrs, pdal::SpatialReference::eCompoundOK);
        BOOST_CHECK(vlrs.size() == 4);
        BOOST_CHECK(vlrs[0].getLength() == boost::uint32_t(96));
    }

    {
        // Now try stripping away the WKT VLR and see that we get the GeoTIFF 
        // derived version instead.
        pdal::drivers::las::VariableLengthRecord::clearVLRs(pdal::drivers::las::VariableLengthRecord::eOGRWKT, vlrs);
        pdal::SpatialReference ref2;
        pdal::drivers::las::VariableLengthRecord::setSRSFromVLRs(vlrs, ref2);

        const std::string wkt2 = "COMPD_CS[\"unknown\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

        const std::string wkt2_ret = ref2.getWKT(pdal::SpatialReference::eCompoundOK);
        BOOST_CHECK(wkt2_ret == wkt2);
    }

    return;
}


BOOST_AUTO_TEST_CASE(test_vertical_datum_notcompound)
{
    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";
    const pdal::SpatialReference srs(wkt);

    std::vector<pdal::drivers::las::VariableLengthRecord> vlrs_compound;
    std::vector<pdal::drivers::las::VariableLengthRecord> vlrs_horizonly;

    pdal::drivers::las::VariableLengthRecord::setVLRsFromSRS(srs, vlrs_compound, pdal::SpatialReference::eCompoundOK);
    pdal::drivers::las::VariableLengthRecord::setVLRsFromSRS(srs, vlrs_horizonly, pdal::SpatialReference::eHorizontalOnly);

    BOOST_CHECK(vlrs_compound.size() == 4);
    BOOST_CHECK(vlrs_horizonly.size() == 4);

    // BUG: the following tests commented out as per ticket #35
    //BOOST_CHECK(vlrs_compound[0].getLength() == 96);
    //BOOST_CHECK(vlrs_compound[1].getLength() == 16);
    //BOOST_CHECK(vlrs_compound[2].getLength() == 21);
    //BOOST_CHECK(vlrs_compound[3].getLength() == 511);
    //BOOST_CHECK(vlrs_horizonly[0].getLength() == 64);
    //BOOST_CHECK(vlrs_horizonly[1].getLength() == 16);
    //BOOST_CHECK(vlrs_horizonly[2].getLength() == 7);
    //BOOST_CHECK(vlrs_horizonly[3].getLength() == 511);

    return;
}

// Try writing a compound coordinate system to file and ensure we get back
// WKT with the geoidgrids (from the WKT VLR).
BOOST_AUTO_TEST_CASE(test_vertical_datums)
{
    std::string tmpfile("tmp_srs.las");
    pdal::Utils::deleteFile(tmpfile);

    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

    {
        pdal::SpatialReference ref;
        ref.setFromUserInput(wkt);
        {
            const std::string wkt2 = ref.getWKT(pdal::SpatialReference::eCompoundOK);
            BOOST_CHECK(wkt == wkt2); // just to make sure
        }

        // Write a very simple file with our SRS and one point.
        pdal::drivers::las::LasReader reader(Support::datapath("1.2-with-color.las"));    

        std::ostream* ofs = pdal::Utils::createFile(tmpfile);
        {
            const boost::uint64_t numPoints = reader.getNumPoints();

            // need to scope the writer, so that's it dtor can use the stream
            pdal::drivers::las::LasWriter writer(reader, *ofs);

            writer.setSpatialReference(ref);

            writer.write(numPoints);
        }
        pdal::Utils::closeFile(ofs);
    }

    // Reopen and check contents. 
    {
        pdal::drivers::las::LasReader reader(tmpfile);

        const pdal::SpatialReference ref2 = reader.getSpatialReference();
        const std::string wkt2 = ref2.getWKT(pdal::SpatialReference::eCompoundOK);
        
        BOOST_CHECK(wkt == wkt2);
    }

    // Cleanup 
    pdal::Utils::deleteFile(tmpfile);

    return;
}


// Try writing only the WKT VLR to a file, and see if the resulting
// file still works ok.
BOOST_AUTO_TEST_CASE(test_writing_vlr)
{
    std::string tmpfile("tmp_srs_9.las");
    pdal::SpatialReference ref;

    const std::string reference_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";

    ref.setFromUserInput( "EPSG:4326" );
    {
        const std::string wkt = ref.getWKT();
        BOOST_CHECK(wkt == reference_wkt);
    }
    {
        std::vector<pdal::drivers::las::VariableLengthRecord> vlrs;
        pdal::drivers::las::VariableLengthRecord::setVLRsFromSRS(ref, vlrs, pdal::SpatialReference::eCompoundOK);
        BOOST_CHECK(vlrs.size() == 4);
        pdal::drivers::las::VariableLengthRecord::clearVLRs(pdal::drivers::las::VariableLengthRecord::eGeoTIFF, vlrs);
        BOOST_CHECK(vlrs.size() == 1);
        pdal::drivers::las::VariableLengthRecord::setSRSFromVLRs(vlrs, ref);
        {
            pdal::SpatialReference srs;
            pdal::drivers::las::VariableLengthRecord::setSRSFromVLRs(vlrs, srs);
            const std::string wkt = srs.getWKT();
            BOOST_CHECK(wkt == reference_wkt);
        }
    }
    {
        const std::string wkt = ref.getWKT();
        BOOST_CHECK(wkt == reference_wkt);
    }

    // Write a very simple file with our SRS and one point.
    {
        pdal::Utils::deleteFile(tmpfile);

        pdal::drivers::las::LasReader readerx(Support::datapath("1.2-with-color.las"));    

        std::ostream* ofs = pdal::Utils::createFile(tmpfile);
        {
            const boost::uint64_t numPoints = readerx.getNumPoints();

            // need to scope the writer, so that's it dtor can use the stream
            pdal::drivers::las::LasWriter writer(readerx, *ofs);

            writer.setSpatialReference(ref);

            writer.write(numPoints);
        }
        pdal::Utils::closeFile(ofs);
    }

    // Reopen and check contents. 
    {
        pdal::drivers::las::LasReader reader(tmpfile);

        pdal::SpatialReference result_ref = reader.getSpatialReference();

        const std::vector<pdal::drivers::las::VariableLengthRecord>& vlrs = reader.getVLRs();
        BOOST_CHECK(vlrs.size() == 4);

        {
            pdal::SpatialReference xxx;
            pdal::drivers::las::VariableLengthRecord::setSRSFromVLRs(vlrs, xxx);
            const std::string wkt = xxx.getWKT();
            BOOST_CHECK(wkt == reference_wkt);
        }

        const std::string wkt = result_ref.getWKT();
        BOOST_CHECK(wkt == reference_wkt);
    }

    // Cleanup 
    pdal::Utils::deleteFile(tmpfile);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
