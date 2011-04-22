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

using namespace libpc;

BOOST_AUTO_TEST_SUITE(SpatialReferenceTest)


BOOST_AUTO_TEST_CASE(test_1)
{
    SpatialReference srs;

    BOOST_CHECK(srs.GetProj4() == "");
    BOOST_CHECK(srs.GetWKT() == "");

    return;
}


#if 0
   utm17_filename(g_test_data_path + "//srs.las")
   utm15_filename(g_test_data_path + "//1.2_3.las")
        
  

    // Test fetching SRS from an existing file
    void to::test<2>()
    {
        std::ifstream ifs;
        ifs.open(utm17_filename.c_str(), std::ios::in | std::ios::binary);
        liblas::Reader reader(ifs);
        
        liblas::Header const& header = reader.GetHeader();
        liblas::SpatialReference const& ref = header.GetSRS();
        
        const char* wkt_c = "PROJCS[\"WGS 84 / UTM zone 17N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-81],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"32617\"]]";
        ensure_equals("WKT comparison", ref.GetWKT(), wkt_c );
        
        const char* proj4_c = "+proj=utm +zone=17 +datum=WGS84 +units=m +no_defs ";
        ensure_equals("Proj.4 comparison", ref.GetProj4(), proj4_c);

    }

    // Test round-tripping proj.4 string
    void to::test<3>()
    {
        liblas::SpatialReference ref;
        const char* proj4_c = "+proj=utm +zone=15 +datum=WGS84 +units=m +no_defs ";
        ref.SetProj4(proj4_c);
        
        ensure_equals("Proj.4 comparison", ref.GetProj4(), proj4_c);
        
    }

    // Test setting EPSG:4326 from User string
    void to::test<4>()
    {
        liblas::SpatialReference ref;
        const char* code = "EPSG:4326";
        const char* proj4_c = "+proj=longlat +datum=WGS84 +no_defs ";
        const char* wkt_c = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
        ref.SetFromUserInput(code);
        
        ensure_equals("Proj.4 comparison", ref.GetProj4(), proj4_c);
        ensure_equals("WKT comparison", ref.GetWKT(), wkt_c );
        
    }

    // Test reprojecting UTM 15 to DD with a filter
    void to::test<5>()
    {
        std::ifstream ifs;
        ifs.open(utm15_filename.c_str(), std::ios::in | std::ios::binary);
        liblas::Reader reader(ifs);
        
        liblas::Header const& header = reader.GetHeader();
        liblas::SpatialReference const& in_ref = header.GetSRS();
        
        const char* utm15_wkt = "PROJCS[\"NAD83 / UTM zone 15N\",GEOGCS[\"NAD83\",DATUM[\"North_American_Datum_1983\",SPHEROID[\"GRS 1980\",6378137,298.2572221010002,AUTHORITY[\"EPSG\",\"7019\"]],AUTHORITY[\"EPSG\",\"6269\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4269\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-93],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AUTHORITY[\"EPSG\",\"26915\"]]";
        ensure_equals("Input WKT comparison", in_ref.GetWKT(), utm15_wkt);

        const char* epsg4326_wkt = "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]]";
        
        liblas::SpatialReference out_ref;
        out_ref.SetWKT(epsg4326_wkt);
        ensure_equals("Output WKT comparison", out_ref.GetWKT(), epsg4326_wkt);
        
        liblas::HeaderPtr out_hdr = liblas::HeaderPtr(new liblas::Header(header));
        out_hdr->SetScale(0.00000001, 0.00000001, 0.01);
        out_hdr->SetOffset(0,0,0);
        liblas::TransformPtr srs_transform = liblas::TransformPtr(new liblas::ReprojectionTransform(in_ref, out_ref, out_hdr));
        
        std::vector<liblas::TransformPtr> transforms;
        transforms.push_back(srs_transform);
        reader.ReadPointAt(0);

        liblas::Point unprojected_point = reader.GetPoint();
        
        ensure_distance("unprojected_point.GetX()", 
                        unprojected_point.GetX(), 
                        double(470692.44), 
                        0.01);

        ensure_distance("unprojected_point.GetY()", 
                        unprojected_point.GetY(), 
                        double(4602888.90), 
                        0.01);
                        
        reader.SetTransforms(transforms);

        // This should throw an out of range exception because the given scale/offset 
        // combination is not sufficient to store the data.
        try
        {
            reader.ReadPointAt(0);
            ensure("std::domain_error was not thrown", false);
        }
        catch (std::domain_error const& e)
        {
            ensure(e.what(), true);
        }
        

        out_hdr->SetScale(0.0000001, 0.0000001, 0.01);
        out_hdr->SetOffset(0,0,0);
        srs_transform = liblas::TransformPtr(new liblas::ReprojectionTransform(in_ref, out_ref, out_hdr));
        
        transforms.clear();
        transforms.push_back(srs_transform);
        reader.SetTransforms(transforms);
        
        reader.Reset();
        reader.ReadPointAt(0);

        
        liblas::Point const& projected_point = reader.GetPoint();

        ensure_distance("projected_point.GetX()", 
                        projected_point.GetX(), 
                        double(-93.35156259), 
                        0.0000001);
        ensure_distance("projected_point.GetY()", 
                        projected_point.GetY(), 
                        double(41.57714839), 
                        0.000001);
        
    }

    // Test VLR sizes from setting SRS
    void to::test<6>()
    {
        liblas::SpatialReference ref;
        const char* code = "EPSG:4326";
        ref.SetFromUserInput(code);
        
        std::vector<liblas::VariableRecord> const& vlrs = ref.GetVLRs();
        ensure_equals("VLR count", vlrs.size(), boost::uint32_t(4));
        ensure_equals("First record size", vlrs[0].GetRecordLength(), boost::uint32_t(64));
        
    }

    // Test incorporation of vertical datum information into WKT string and
    // into GeoTIFF VLRs. 
    void to::test<7>()
    {
        liblas::SpatialReference ref;
        const char* wkt_c = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";
        const char* exp_gtiff = "Geotiff_Information:\n   Version: 1\n   Key_Revision: 1.0\n   Tagged_Information:\n      End_Of_Tags.\n   Keyed_Information:\n      GTRasterTypeGeoKey (Short,1): RasterPixelIsArea\n      GTModelTypeGeoKey (Short,1): ModelTypeGeographic\n      GeogAngularUnitsGeoKey (Short,1): Angular_Degree\n      GeogCitationGeoKey (Ascii,7): \"WGS 84\"\n      GeographicTypeGeoKey (Short,1): GCS_WGS_84\n      GeogInvFlatteningGeoKey (Double,1): 298.257223563    \n      GeogSemiMajorAxisGeoKey (Double,1): 6378137          \n      VerticalCitationGeoKey (Ascii,14): \"NAVD88 height\"\n      VerticalCSTypeGeoKey (Short,1): Unknown-5703\n      VerticalDatumGeoKey (Short,1): Unknown-5103\n      VerticalUnitsGeoKey (Short,1): Linear_Meter\n      End_Of_Keys.\n   End_Of_Geotiff.\n";

        ref.SetFromUserInput(wkt_c);

        ensure_equals("WKT comparison", ref.GetWKT(liblas::SpatialReference::eCompoundOK), wkt_c );
        
        std::vector<liblas::VariableRecord> const& vlrs = ref.GetVLRs();
        ensure_equals("VLR count", vlrs.size(), boost::uint32_t(4));
        ensure_equals("First record size", vlrs[0].GetRecordLength(), boost::uint32_t(96));

        liblas::property_tree::ptree tree = ref.GetPTree();
        std::string gtiff = tree.get<std::string>("gtiff");

        ensure_equals("GeoTIFF Tags", gtiff, exp_gtiff );

        // Now try stripping away the WKT VLR and see that we get the GeoTIFF 
        // derived version instead.
        ref.ClearVLRs( liblas::SpatialReference::eOGRWKT );

        wkt_c = "COMPD_CS[\"unknown\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0],UNIT[\"degree\",0.0174532925199433],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";
        ensure_equals("non OGR WKT comparison", ref.GetWKT(liblas::SpatialReference::eCompoundOK), wkt_c );
    }

    // Try writing a compound coordinate system to file and ensure we get back
    // WKT with the geoidgrids (from the WKT VLR).
    void to::test<8>()
    {
        std::string tmpfile_(g_test_data_path + "//tmp_srs.las");
        liblas::SpatialReference ref, result_ref;
        const char* wkt_c = "COMPD_CS[\"WGS 84 + VERT_CS\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],VERT_CS[\"NAVD88 height\",VERT_DATUM[\"North American Vertical Datum 1988\",2005,AUTHORITY[\"EPSG\",\"5103\"],EXTENSION[\"PROJ4_GRIDS\",\"g2003conus.gtx,g2003alaska.gtx,g2003h01.gtx,g2003p01.gtx\"]],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Up\",UP],AUTHORITY[\"EPSG\",\"5703\"]]]";

        ref.SetFromUserInput( wkt_c );

        // Write a very simple file with our SRS and one point.
        {
            std::ofstream ofs;
            ofs.open(tmpfile_.c_str(), std::ios::out | std::ios::binary);

            liblas::Header header;
            header.SetSRS( ref );

            liblas::Writer writer(ofs, header);
            
            liblas::Point point;
            point.SetCoordinates( -117, 33, 100 );
            writer.WritePoint( point );
        }

        // Reopen and check contents. 
        {
            std::ifstream ifs;
            ifs.open( tmpfile_.c_str(), std::ios::in | std::ios::binary );
            
            liblas::Reader reader(ifs);

            liblas::Header const& header = reader.GetHeader();
            
            result_ref = header.GetSRS();
        }

        ensure_equals("WKT comparison", ref.GetWKT(liblas::SpatialReference::eCompoundOK), wkt_c );

        // Cleanup 
        std::remove(tmpfile_.c_str());
    }

    // Try writing only the WKT VLR to a file, and see if the resulting
    // file still works ok.
    void to::test<9>()
    {
        std::string tmpfile_(g_test_data_path + "//tmp_srs_9.las");
        liblas::SpatialReference ref, result_ref;

        ref.SetFromUserInput( "EPSG:4326" );
        ref.ClearVLRs( liblas::SpatialReference::eGeoTIFF );

        // Write a very simple file with our SRS and one point.
        {
            std::ofstream ofs;
            ofs.open(tmpfile_.c_str(), std::ios::out | std::ios::binary);

            liblas::Header header;
            header.SetSRS( ref );

            liblas::Writer writer(ofs, header);
            
            liblas::Point point;
            point.SetCoordinates( -117, 33, 100 );
            writer.WritePoint( point );
        }

        // Reopen and check contents. 
        {
            std::ifstream ifs;
            ifs.open( tmpfile_.c_str(), std::ios::in | std::ios::binary );
            
            liblas::Reader reader(ifs);

            liblas::Header const& header = reader.GetHeader();
            
            result_ref = header.GetSRS();
        }

        std::vector<liblas::VariableRecord> const& vlrs = ref.GetVLRs();
        ensure_equals("VLR count", vlrs.size(), boost::uint32_t(1));

        liblas::property_tree::ptree tree = ref.GetPTree();
        std::string gtiff = tree.get<std::string>("gtiff");

        // there should be no geotiff definition.
        ensure_equals("GeoTIFF Tags", gtiff, "" );

        // Cleanup 
        std::remove(tmpfile_.c_str());
    }

}

#endif

BOOST_AUTO_TEST_SUITE_END()
