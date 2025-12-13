/******************************************************************************
* Copyright (c) 2015, Peter J. Gadomski <pete.gadomski@gmail.com>
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

#include <filters/GeoreferenceFilter.hpp>
#include <filters/ReprojectionFilter.hpp>
#include <filters/TransformationFilter.hpp>
#include <io/BpfWriter.hpp>
#include <io/BufferReader.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/pdal_test_main.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/Georeference.hpp>

#include "Support.hpp"

#include <cmath>

namespace pdal
{
namespace georeference
{


TEST(RotationMatrix, Constructor)
{
    RotationMatrix matrix(1, 0, 0, 0, 1, 0, 0, 0, 1);
    EXPECT_DOUBLE_EQ(1, matrix.m00);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m10);
    EXPECT_DOUBLE_EQ(1, matrix.m11);
    EXPECT_DOUBLE_EQ(0, matrix.m12);
    EXPECT_DOUBLE_EQ(0, matrix.m20);
    EXPECT_DOUBLE_EQ(0, matrix.m21);
    EXPECT_DOUBLE_EQ(1, matrix.m22);
}


TEST(RotationMatrix, IdentityMatrix)
{
    RotationMatrix matrix = createIdentityMatrix();
    EXPECT_DOUBLE_EQ(1, matrix.m00);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m01);
    EXPECT_DOUBLE_EQ(0, matrix.m10);
    EXPECT_DOUBLE_EQ(1, matrix.m11);
    EXPECT_DOUBLE_EQ(0, matrix.m12);
    EXPECT_DOUBLE_EQ(0, matrix.m20);
    EXPECT_DOUBLE_EQ(0, matrix.m21);
    EXPECT_DOUBLE_EQ(1, matrix.m22);
}


TEST(Georeference, Zeros)
{
    Xyz point = georeferenceWgs84(0, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(0, 0, 0));
    EXPECT_DOUBLE_EQ(0, point.X);
    EXPECT_DOUBLE_EQ(0, point.Y);
    EXPECT_DOUBLE_EQ(0, point.Z);
}


TEST(Georeference, LatLonElev)
{
    Xyz point = georeferenceWgs84(0, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}


TEST(Georeference, Range)
{
    Xyz point = georeferenceWgs84(3, 0, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(0, point.Z);
}


TEST(Georeference, RangeAndAngle)
{
    Xyz point = georeferenceWgs84(3, M_PI / 2, createIdentityMatrix(),
                                  createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(0.9999988728659957, point.X);
    EXPECT_DOUBLE_EQ(2, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}


TEST(Georeference, WithImu)
{
    RotationMatrix imuMatrix(0, 1, 0, 0, 0, -1, -1, 0, 0);
    Xyz point =
        georeferenceWgs84(3, 0, imuMatrix, createIdentityMatrix(), Xyz(1, 2, 3));
    EXPECT_DOUBLE_EQ(1, point.X);
    EXPECT_DOUBLE_EQ(2.0000004696006983, point.Y);
    EXPECT_DOUBLE_EQ(3, point.Z);
}
} // namespace georeference
} // namespace pdal

namespace pdal
{
namespace
{

using Transform = TransformationFilter::Transform;
using TransformArray = Transform::ArrayType;

Transform identityTransform()
{
    TransformArray arr{1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0};
    return Transform(arr);
}

std::string writeTrajectoryFile()
{
    using DimId = Dimension::Id;

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::GpsTime);
    layout->registerDim(DimId::Roll);
    layout->registerDim(DimId::Pitch);
    layout->registerDim(DimId::Azimuth);
    layout->registerDim(DimId::WanderAngle);
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);

    // WGS84 trajectory (EPSG:4979)
    // X = longitude in radians, Y = latitude in radians, Z = altitude in meters
    // Approximate position: Paris, France (2.3522° E, 48.8566° N, 50m alt)
    const double lon_rad = 0.041015;  // ~2.35° in radians
    const double lat_rad = 0.852478;  // ~48.85° in radians
    const double alt_m = 50.0;

    PointViewPtr view(new PointView(table));
    view->setField(DimId::GpsTime, 0, 0.0);
    view->setField(DimId::Roll, 0, 0.0);
    view->setField(DimId::Pitch, 0, 0.0);
    view->setField(DimId::Azimuth, 0, 0.0);
    view->setField(DimId::WanderAngle, 0, 0.0);
    view->setField(DimId::X, 0, lon_rad);
    view->setField(DimId::Y, 0, lat_rad);
    view->setField(DimId::Z, 0, alt_m);

    view->setField(DimId::GpsTime, 1, 10.0);
    view->setField(DimId::Roll, 1, 0.0);
    view->setField(DimId::Pitch, 1, 0.0);
    view->setField(DimId::Azimuth, 1, 0.0);
    view->setField(DimId::WanderAngle, 1, 0.0);
    view->setField(DimId::X, 1, lon_rad);
    view->setField(DimId::Y, 1, lat_rad);
    view->setField(DimId::Z, 1, alt_m);

    BufferReader reader;
    reader.addView(view);

    std::string filename = Support::temppath("georeference_trajectory.bpf");
    FileUtils::deleteFile(filename);

    Options writerOpts;
    writerOpts.add("filename", filename);

    BpfWriter writer;
    writer.setOptions(writerOpts);
    writer.setInput(reader);
    writer.prepare(table);
    writer.execute(table);
    return filename;
}

} // unnamed namespace

TEST(GeoreferenceFilterTest, MissingBeamDimensionsThrows)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, 0.0);
    view->setField(DimId::Y, 0, 0.0);
    view->setField(DimId::Z, 0, 0.0);
    view->setField(DimId::GpsTime, 0, 5.0);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    opts.add("transform_beam", true);
    filter.setOptions(opts);
    filter.setInput(reader);

    EXPECT_THROW(filter.prepare(table), pdal_error);

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, TransformsPointAndBeamDirection)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);
    layout->registerDim(DimId::BeamOriginX);
    layout->registerDim(DimId::BeamOriginY);
    layout->registerDim(DimId::BeamOriginZ);
    layout->registerDim(DimId::BeamDirectionX);
    layout->registerDim(DimId::BeamDirectionY);
    layout->registerDim(DimId::BeamDirectionZ);

    // Point in scanner reference frame (local coordinates)
    const double scanX = 1.5;
    const double scanY = 2.0;
    const double scanZ = -0.5;
    const double beamOriginX = 0.2;
    const double beamOriginY = 0.3;
    const double beamOriginZ = 0.1;
    
    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, scanX);
    view->setField(DimId::Y, 0, scanY);
    view->setField(DimId::Z, 0, scanZ);
    view->setField(DimId::GpsTime, 0, 5.0);
    view->setField(DimId::BeamOriginX, 0, beamOriginX);
    view->setField(DimId::BeamOriginY, 0, beamOriginY);
    view->setField(DimId::BeamOriginZ, 0, beamOriginZ);
    view->setField(DimId::BeamDirectionX, 0, 1.0);
    view->setField(DimId::BeamDirectionY, 0, 0.0);
    view->setField(DimId::BeamDirectionZ, 0, 0.0);

    // Calculate distance in scanner reference frame
    double dx_scanner = scanX - beamOriginX;
    double dy_scanner = scanY - beamOriginY;
    double dz_scanner = scanZ - beamOriginZ;
    double dist_scanner = std::sqrt(dx_scanner*dx_scanner + dy_scanner*dy_scanner + dz_scanner*dz_scanner);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    opts.add("transform_beam", true);
    filter.setOptions(opts);
    filter.setInput(reader);

    filter.prepare(table);
    PointViewSet views = filter.execute(table);
    ASSERT_EQ(views.size(), 1u);
    PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), 1u);

    PointRef point(*result, 0);
    // Points should now be georeferenced (WGS84 coordinates)
    double lon = point.getFieldAs<double>(DimId::X);
    double lat = point.getFieldAs<double>(DimId::Y);
    double alt = point.getFieldAs<double>(DimId::Z);
    
    EXPECT_TRUE(std::isfinite(lon));
    EXPECT_TRUE(std::isfinite(lat));
    EXPECT_TRUE(std::isfinite(alt));
    
    double beamOriginLon = point.getFieldAs<double>(DimId::BeamOriginX);
    double beamOriginLat = point.getFieldAs<double>(DimId::BeamOriginY);
    double beamOriginAlt = point.getFieldAs<double>(DimId::BeamOriginZ);
    
    EXPECT_TRUE(std::isfinite(beamOriginLon));
    EXPECT_TRUE(std::isfinite(beamOriginLat));
    EXPECT_TRUE(std::isfinite(beamOriginAlt));
    
    // BeamDirection must remain a unit vector
    double dirX = point.getFieldAs<double>(DimId::BeamDirectionX);
    double dirY = point.getFieldAs<double>(DimId::BeamDirectionY);
    double dirZ = point.getFieldAs<double>(DimId::BeamDirectionZ);
    double norm = std::sqrt(dirX*dirX + dirY*dirY + dirZ*dirZ);
    EXPECT_NEAR(norm, 1.0, 1e-6);

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, PreservesDistancesBetweenPoints)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Two points in scanner reference frame
    const double point1X = 1.5;
    const double point1Y = 2.0;
    const double point1Z = -0.5;
    
    const double point2X = 3.2;
    const double point2Y = 1.8;
    const double point2Z = 0.3;
    
    const double gpsTime = 5.0;

    PointViewPtr view(new PointView(table));
    
    // First point
    view->setField(DimId::X, 0, point1X);
    view->setField(DimId::Y, 0, point1Y);
    view->setField(DimId::Z, 0, point1Z);
    view->setField(DimId::GpsTime, 0, gpsTime);
    
    // Second point
    view->setField(DimId::X, 1, point2X);
    view->setField(DimId::Y, 1, point2Y);
    view->setField(DimId::Z, 1, point2Z);
    view->setField(DimId::GpsTime, 1, gpsTime);

    // Calculate distance between the two points in scanner reference frame
    double dx_scanner = point2X - point1X;
    double dy_scanner = point2Y - point1Y;
    double dz_scanner = point2Z - point1Z;
    double dist_scanner = std::sqrt(dx_scanner*dx_scanner + dy_scanner*dy_scanner + dz_scanner*dz_scanner);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    filter.setOptions(opts);
    filter.setInput(reader);

    filter.prepare(table);
    PointViewSet views = filter.execute(table);
    ASSERT_EQ(views.size(), 1u);
    PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), 2u);

    // Get georeferenced coordinates of both points
    PointRef p1(*result, 0);
    double lon1 = p1.getFieldAs<double>(DimId::X);
    double lat1 = p1.getFieldAs<double>(DimId::Y);
    double alt1 = p1.getFieldAs<double>(DimId::Z);
    
    PointRef p2(*result, 1);
    double lon2 = p2.getFieldAs<double>(DimId::X);
    double lat2 = p2.getFieldAs<double>(DimId::Y);
    double alt2 = p2.getFieldAs<double>(DimId::Z);

    EXPECT_TRUE(std::isfinite(lon1) && std::isfinite(lat1) && std::isfinite(alt1));
    EXPECT_TRUE(std::isfinite(lon2) && std::isfinite(lat2) && std::isfinite(alt2));

    // Convert WGS84 → ECEF for both points
    // Use filters.reprojection to convert from WGS84 geographic (EPSG:4979) to ECEF (EPSG:4978)
    BufferReader wgs84Reader;
    wgs84Reader.addView(result);

    ReprojectionFilter reprojFilter;
    Options reprojOpts;
    reprojOpts.add("in_srs", "EPSG:4979");
    reprojOpts.add("out_srs", "EPSG:4978");
    reprojFilter.setOptions(reprojOpts);
    reprojFilter.setInput(wgs84Reader);

    PointTable ecefTable;
    reprojFilter.prepare(ecefTable);
    PointViewSet ecefViews = reprojFilter.execute(ecefTable);
    ASSERT_EQ(ecefViews.size(), 1u);
    PointViewPtr ecefView = *ecefViews.begin();
    ASSERT_EQ(ecefView->size(), 2u);

    // Get ECEF coordinates for both points
    PointRef ecefP1(*ecefView, 0);
    double ecef1X = ecefP1.getFieldAs<double>(DimId::X);
    double ecef1Y = ecefP1.getFieldAs<double>(DimId::Y);
    double ecef1Z = ecefP1.getFieldAs<double>(DimId::Z);

    PointRef ecefP2(*ecefView, 1);
    double ecef2X = ecefP2.getFieldAs<double>(DimId::X);
    double ecef2Y = ecefP2.getFieldAs<double>(DimId::Y);
    double ecef2Z = ecefP2.getFieldAs<double>(DimId::Z);
    
    // Calculate distance between the two points in ECEF
    double dx_ecef = ecef2X - ecef1X;
    double dy_ecef = ecef2Y - ecef1Y;
    double dz_ecef = ecef2Z - ecef1Z;
    double dist_ecef = std::sqrt(dx_ecef*dx_ecef + dy_ecef*dy_ecef + dz_ecef*dz_ecef);
    
    // Distance between the two points must be preserved
    EXPECT_NEAR(dist_ecef, dist_scanner, 1e-3);  // Tolerance: 1mm

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, ForwardAndReverseRoundtrip)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Point in scanner reference frame
    const double scanX = 1.5;
    const double scanY = 2.5;
    const double scanZ = -0.5;
    const double gpsTime = 5.0;

    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, scanX);
    view->setField(DimId::Y, 0, scanY);
    view->setField(DimId::Z, 0, scanZ);
    view->setField(DimId::GpsTime, 0, gpsTime);

    BufferReader reader;
    reader.addView(view);

    // Step 1: Forward (scanner → georeferenced)
    GeoreferenceFilter forwardFilter;
    Options forwardOpts;
    forwardOpts.add("trajectory_file", trajFile);
    forwardOpts.add("scan2imu", identityTransform());
    forwardFilter.setOptions(forwardOpts);
    forwardFilter.setInput(reader);

    PointTable table1;
    forwardFilter.prepare(table1);
    PointViewSet views1 = forwardFilter.execute(table1);
    ASSERT_EQ(views1.size(), 1u);
    PointViewPtr georefView = *views1.begin();
    ASSERT_EQ(georefView->size(), 1u);

    // Get georeferenced coordinates
    PointRef georefPoint(*georefView, 0);
    double georefX = georefPoint.getFieldAs<double>(DimId::X);
    double georefY = georefPoint.getFieldAs<double>(DimId::Y);
    double georefZ = georefPoint.getFieldAs<double>(DimId::Z);

    EXPECT_TRUE(std::isfinite(georefX));
    EXPECT_TRUE(std::isfinite(georefY));
    EXPECT_TRUE(std::isfinite(georefZ));

    // Step 2: Reverse (georeferenced → scanner)
    BufferReader georefReader;
    georefReader.addView(georefView);

    GeoreferenceFilter reverseFilter;
    Options reverseOpts;
    reverseOpts.add("trajectory_file", trajFile);
    reverseOpts.add("scan2imu", identityTransform());
    reverseOpts.add("reverse", true);
    reverseFilter.setOptions(reverseOpts);
    reverseFilter.setInput(georefReader);

    PointTable table2;
    reverseFilter.prepare(table2);
    PointViewSet views2 = reverseFilter.execute(table2);
    ASSERT_EQ(views2.size(), 1u);
    PointViewPtr scanView = *views2.begin();
    ASSERT_EQ(scanView->size(), 1u);

    // Step 3: Verify that we get back the original scanner coordinates
    PointRef scanPoint(*scanView, 0);
    double resultX = scanPoint.getFieldAs<double>(DimId::X);
    double resultY = scanPoint.getFieldAs<double>(DimId::Y);
    double resultZ = scanPoint.getFieldAs<double>(DimId::Z);

    EXPECT_NEAR(resultX, scanX, 1e-3);
    EXPECT_NEAR(resultY, scanY, 1e-3);
    EXPECT_NEAR(resultZ, scanZ, 1e-3);

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, ENUCoordinateSystem)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Point in scanner reference frame
    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, 1.0);
    view->setField(DimId::Y, 0, 2.0);
    view->setField(DimId::Z, 0, 3.0);
    view->setField(DimId::GpsTime, 0, 5.0);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    opts.add("coordinate_system", "ENU");
    filter.setOptions(opts);
    filter.setInput(reader);

    filter.prepare(table);
    PointViewSet views = filter.execute(table);
    ASSERT_EQ(views.size(), 1u);
    PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), 1u);

    PointRef point(*result, 0);
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::X)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Y)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Z)));

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, InvalidCoordinateSystemThrows)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Point in scanner reference frame
    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, 1.0);
    view->setField(DimId::Y, 0, 2.0);
    view->setField(DimId::Z, 0, 3.0);
    view->setField(DimId::GpsTime, 0, 5.0);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    opts.add("coordinate_system", "INVALID");
    filter.setOptions(opts);
    filter.setInput(reader);

    EXPECT_THROW(filter.prepare(table), pdal_error);

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, WithTimeOffset)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Point in scanner reference frame
    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, 1.0);
    view->setField(DimId::Y, 0, 2.0);
    view->setField(DimId::Z, 0, 3.0);
    view->setField(DimId::GpsTime, 0, 3.0);

    BufferReader reader;
    reader.addView(view);

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", identityTransform());
    opts.add("time_offset", 2.0);
    filter.setOptions(opts);
    filter.setInput(reader);

    filter.prepare(table);
    PointViewSet views = filter.execute(table);
    ASSERT_EQ(views.size(), 1u);
    PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), 1u);

    PointRef point(*result, 0);
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::X)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Y)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Z)));

    FileUtils::deleteFile(trajFile);
}

TEST(GeoreferenceFilterTest, WithCustomScan2ImuTransform)
{
    using DimId = Dimension::Id;

    std::string trajFile = writeTrajectoryFile();

    PointTable table;
    PointLayoutPtr layout = table.layout();
    layout->registerDim(DimId::X);
    layout->registerDim(DimId::Y);
    layout->registerDim(DimId::Z);
    layout->registerDim(DimId::GpsTime);

    // Point in scanner reference frame
    PointViewPtr view(new PointView(table));
    view->setField(DimId::X, 0, 1.0);
    view->setField(DimId::Y, 0, 0.0);
    view->setField(DimId::Z, 0, 0.0);
    view->setField(DimId::GpsTime, 0, 5.0);

    BufferReader reader;
    reader.addView(view);

    // 90-degree rotation around Z axis
    TransformArray rotationTransform = {
        0.0, -1.0, 0.0, 0.0,
        1.0,  0.0, 0.0, 0.0,
        0.0,  0.0, 1.0, 0.0,
        0.0,  0.0, 0.0, 1.0
    };

    GeoreferenceFilter filter;
    Options opts;
    opts.add("trajectory_file", trajFile);
    opts.add("scan2imu", Transform(rotationTransform));
    filter.setOptions(opts);
    filter.setInput(reader);

    filter.prepare(table);
    PointViewSet views = filter.execute(table);
    ASSERT_EQ(views.size(), 1u);
    PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), 1u);

    PointRef point(*result, 0);
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::X)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Y)));
    EXPECT_TRUE(std::isfinite(point.getFieldAs<double>(DimId::Z)));

    FileUtils::deleteFile(trajFile);
}

} // namespace pdal
