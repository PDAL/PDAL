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

#include <libpc/SpatialReference.hpp>
#include <libpc/Utils.hpp>
#include <libpc/drivers/las/VariableLengthRecord.hpp>
#include <libpc/drivers/las/Writer.hpp>
#include <libpc/drivers/las/Reader.hpp>

#include "Support.hpp"

BOOST_AUTO_TEST_SUITE(SpatialReferenceTest)


BOOST_AUTO_TEST_CASE(test_env_vars)
{
    
#ifdef _MSC_VER
    const char* gdal_data = getenv("GDAL_DATA");
    const char* proj_lib = getenv("PROJ_LIB");

    BOOST_CHECK(libpc::Utils::fileExists(gdal_data));
    BOOST_CHECK(libpc::Utils::fileExists(proj_lib));
#endif
    return;
}


BOOST_AUTO_TEST_CASE(test_ctor)
{
    libpc::SpatialReference srs;

    BOOST_CHECK(srs.getProj4() == "");
    BOOST_CHECK(srs.getWKT() == "");

    return;
}


// Test round-tripping proj.4 string
BOOST_AUTO_TEST_CASE(test_proj4_roundtrip)
{
    libpc::SpatialReference ref;
    
    const std::string proj4 = "+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs ";
    const std::string proj4_ellps = "+proj=utm +zone=15 +ellps=WGS84 +datum=WGS84 +units=m +no_defs ";

    ref.setProj4(proj4);
    const std::string ret = ref.getProj4();
    BOOST_CHECK(ret == proj4);

    ref.setProj4(proj4_ellps);
    const std::string ret2 = ref.getProj4();
    BOOST_CHECK(ret == proj4);

    return;
}


// Test setting EPSG:4326 from User string
BOOST_AUTO_TEST_CASE(test_userstring_roundtrip)
{
    libpc::SpatialReference ref;

    const std::string code = "EPSG:4326";
    const std::string proj4 = "+proj=longlat +datum=WGS84 +no_defs ";
    const std::string proj4_ellps = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs ";
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
    libpc::drivers::las::LasReader reader(Support::datapath("utm17.las"));

    const libpc::SpatialReference& ref = reader.getSpatialReference();

    BOOST_CHECK(reader.getVLRs().size() == 3);

    const std::string ret_wkt = ref.getWKT();
    const std::string ret_proj4 = ref.getProj4();

    const std::string wkt = "PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-81],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"32617\"]]";
    BOOST_CHECK(ret_wkt == wkt);

    const std::string proj4 = "+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs ";
    BOOST_CHECK(ret_proj4 == proj4);

    return;
}
    

// Test VLR sizes from setting SRS
BOOST_AUTO_TEST_CASE(test_vlr_sizes)
{
    libpc::SpatialReference ref;
    const char* code = "EPSG:4326";
    ref.setFromUserInput(code);

    std::vector<libpc::drivers::las::VariableLengthRecord> vlrs;
    libpc::drivers::las::VariableLengthRecord::setVLRsFromSRS_X(ref, vlrs);

    BOOST_CHECK(vlrs.size() == boost::uint32_t(4));
    BOOST_CHECK(vlrs[0].getLength() == boost::uint32_t(64));

    return;
}


// Test incorporation of vertical datum information into WKT string and
// into GeoTIFF VLRs. 
BOOST_AUTO_TEST_CASE(test_vertical_datum)
{
    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";
    const std::string exp_gtiff = "Geotiff_Information:\n   Version: 1\n   Key_Revision: 1.0\n   Tagged_Information:\n      End_Of_Tags.\n   Keyed_Information:\n      GTRasterTypeGeoKey (Short,1): RasterPixelIsArea\n      GTModelTypeGeoKey (Short,1): ModelTypeGeographic\n      GeogAngularUnitsGeoKey (Short,1): Angular_Degree\n      GeogCitationGeoKey (Ascii,7): \"WGS 84\"\n      GeographicTypeGeoKey (Short,1): GCS_WGS_84\n      GeogInvFlatteningGeoKey (Double,1): 298.257223563    \n      GeogSemiMajorAxisGeoKey (Double,1): 6378137          \n      VerticalCitationGeoKey (Ascii,14): \"NAVD88 height\"\n      VerticalCSTypeGeoKey (Short,1): Unknown-5703\n      VerticalDatumGeoKey (Short,1): Unknown-5103\n      VerticalUnitsGeoKey (Short,1): Linear_Meter\n      End_Of_Keys.\n   End_Of_Geotiff.\n";

    libpc::SpatialReference ref;
    {
        ref.setFromUserInput(wkt);
        BOOST_CHECK(ref.getWKT(libpc::SpatialReference::eCompoundOK) == wkt);
    }

    std::vector<libpc::drivers::las::VariableLengthRecord> vlrs;
    {
        libpc::drivers::las::VariableLengthRecord::setVLRsFromSRS_X(ref, vlrs);
        BOOST_CHECK(vlrs.size() == 4);
        BOOST_CHECK(vlrs[0].getLength() == boost::uint32_t(96));
    }

    {
        boost::property_tree::ptree tree = ref.getPTree();
        std::string gtiff = tree.get<std::string>("gtiff");

        BOOST_CHECK(gtiff == exp_gtiff);
    }

    {
        // Now try stripping away the WKT VLR and see that we get the GeoTIFF 
        // derived version instead.
        libpc::drivers::las::VariableLengthRecord::clearVLRs(libpc::drivers::las::VariableLengthRecord::eOGRWKT, vlrs);
        libpc::SpatialReference ref2;
        libpc::drivers::las::VariableLengthRecord::setSRSFromVLRs_X(vlrs, ref2);

        const std::string wkt2 = "COMPD_CS[\"unknown\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

        const std::string wkt2_ret = ref2.getWKT(libpc::SpatialReference::eCompoundOK);
        BOOST_CHECK(wkt2_ret == wkt2);
    }

    return;
}


// Try writing a compound coordinate system to file and ensure we get back
// WKT with the geoidgrids (from the WKT VLR).
BOOST_AUTO_TEST_CASE(test_vertical_datums)
{
    std::string tmpfile("tmp_srs.las");
    libpc::Utils::deleteFile(tmpfile);

    const std::string wkt = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

    {
        libpc::SpatialReference ref;
        ref.setFromUserInput(wkt);
        {
            const std::string wkt2 = ref.getWKT(libpc::SpatialReference::eCompoundOK);
            BOOST_CHECK(wkt == wkt2); // just to make sure
        }

        // Write a very simple file with our SRS and one point.
        libpc::drivers::las::LasReader reader(Support::datapath("1.2-with-color.las"));    

        std::ostream* ofs = libpc::Utils::createFile(tmpfile);
        {
            const boost::uint64_t numPoints = reader.getNumPoints();

            // need to scope the writer, so that's it dtor can use the stream
            libpc::drivers::las::LasWriter writer(reader, *ofs);

            writer.setSpatialReference(ref);

            writer.write(numPoints);
        }
        libpc::Utils::closeFile(ofs);
    }

    // Reopen and check contents. 
    {
        libpc::drivers::las::LasReader reader(tmpfile);

        const libpc::SpatialReference ref2 = reader.getSpatialReference();
        const std::string wkt2 = ref2.getWKT(libpc::SpatialReference::eCompoundOK);
        
        BOOST_CHECK(wkt == wkt2); // fails
    }

    // Cleanup 
    libpc::Utils::deleteFile(tmpfile);

    return;
}


// Try writing only the WKT VLR to a file, and see if the resulting
// file still works ok.
BOOST_AUTO_TEST_CASE(test_writing_vlr)
{
    std::string tmpfile("tmp_srs_9.las");
    libpc::SpatialReference ref;

    const std::string reference_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";

    ref.setFromUserInput( "EPSG:4326" );
    {
        const std::string wkt = ref.getWKT();
        BOOST_CHECK(wkt == reference_wkt);
    }
    {
        std::vector<libpc::drivers::las::VariableLengthRecord> vlrs;
        libpc::drivers::las::VariableLengthRecord::setVLRsFromSRS_X(ref, vlrs);
        BOOST_CHECK(vlrs.size() == 4);
        libpc::drivers::las::VariableLengthRecord::clearVLRs(libpc::drivers::las::VariableLengthRecord::eGeoTIFF, vlrs);
        BOOST_CHECK(vlrs.size() == 1);
        libpc::drivers::las::VariableLengthRecord::setSRSFromVLRs_X(vlrs, ref);
        {
            libpc::SpatialReference xxx;
            libpc::drivers::las::VariableLengthRecord::setSRSFromVLRs_X(vlrs, xxx);
            const std::string wkt = xxx.getWKT();
            BOOST_CHECK(wkt == ""); // BUG: shouldn't this be equal to reference_wkt (as in the next test immediately below us?)
        }
    }
    {
        const std::string wkt = ref.getWKT();
        BOOST_CHECK(wkt == reference_wkt);
    }

    // Write a very simple file with our SRS and one point.
    {
        libpc::Utils::deleteFile(tmpfile);

        libpc::drivers::las::LasReader readerx(Support::datapath("1.2-with-color.las"));    

        std::ostream* ofs = libpc::Utils::createFile(tmpfile);
        {
            const boost::uint64_t numPoints = readerx.getNumPoints();

            // need to scope the writer, so that's it dtor can use the stream
            libpc::drivers::las::LasWriter writer(readerx, *ofs);

            writer.setSpatialReference(ref);

            writer.write(numPoints);
        }
        libpc::Utils::closeFile(ofs);
    }

    // Reopen and check contents. 
    {
        libpc::drivers::las::LasReader reader(tmpfile);

        libpc::SpatialReference result_ref = reader.getSpatialReference();

        const std::vector<libpc::drivers::las::VariableLengthRecord>& vlrs = reader.getVLRs();
        BOOST_CHECK(vlrs.size() == 1);

        {
            libpc::SpatialReference xxx;
            libpc::drivers::las::VariableLengthRecord::setSRSFromVLRs_X(vlrs, xxx);
            const std::string wkt = xxx.getWKT();
            BOOST_CHECK(wkt == "");
        }

        const std::string wkt = result_ref.getWKT();
        BOOST_CHECK(wkt == "");

        boost::property_tree::ptree tree = ref.getPTree();
        std::string gtiff = tree.get<std::string>("gtiff");

        // there should be no geotiff definition.
        BOOST_CHECK(gtiff == "Geotiff_Information:\n   Version: 1\n   Key_Revision: 1.0\n   Tagged_Information:\n      End_Of_Tags.\n   Keyed_Information:\n      End_Of_Keys.\n   End_Of_Geotiff.\n");
    }

    // Cleanup 
    libpc::Utils::deleteFile(tmpfile);

    return;
}


BOOST_AUTO_TEST_SUITE_END()
