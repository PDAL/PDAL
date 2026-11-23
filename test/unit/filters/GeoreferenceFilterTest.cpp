/******************************************************************************
* Copyright (c) 2025, Guilhem Villemin <guilhem.villemin@altametris.com>
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
#include <filters/private/georeference/Trajectory.hpp>
#include <filters/private/georeference/Utils.hpp>
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

std::string writeTrajectoryFile(double endAltitude = 50.0)
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
    view->setField(DimId::Z, 1, endAltitude);

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

TEST(GeoreferenceFilterTest, ScalarInterpolation)
{
    using DimId = Dimension::Id;
    PointTable table;
    table.layout()->registerDim(DimId::Z);
    PointViewPtr view(new PointView(table));
    view->setField(DimId::Z, 0, 50.0);
    view->setField(DimId::Z, 1, 150.0);
    PointRef first(*view, 0);
    PointRef second(*view, 1);

    for (double fraction : {0.0, 0.25, 0.5, 0.75, 1.0})
    {
        SCOPED_TRACE(fraction);
        const double expected = 50.0 + 100.0 * fraction;
        EXPECT_DOUBLE_EQ(georeference::Utils::getValue(50.0, 150.0, fraction),
            expected);
        EXPECT_DOUBLE_EQ(georeference::Utils::getValue(first, second, DimId::Z,
            fraction), expected);
        EXPECT_DOUBLE_EQ(georeference::Utils::getValue(view, DimId::Z, 1,
            fraction), expected);
    }
}

TEST(GeoreferenceFilterTest, InterpolatesTrajectoryAltitudeAndTime)
{
    const std::string filename = writeTrajectoryFile(150.0);
    {
        georeference::Trajectory trajectory(filename, NL::json::object());
        for (double time : {0.1, 2.5, 5.0, 7.5, 10.0})
        {
            SCOPED_TRACE(time);
            georeference::TrajPoint point;
            ASSERT_TRUE(trajectory.getTrajPoint(time, point));
            EXPECT_DOUBLE_EQ(point.time, time);
            EXPECT_NEAR(point.z, 50.0 + 10.0 * time, 1e-10);
        }
    }
    FileUtils::deleteFile(filename);
}

TEST(GeoreferenceFilterTest, GeoreferencesWithInterpolatedAltitude)
{
    using DimId = Dimension::Id;
    const std::string filename = writeTrajectoryFile(150.0);
    PointTable table;
    table.layout()->registerDims({DimId::X, DimId::Y, DimId::Z, DimId::GpsTime});
    PointViewPtr view(new PointView(table));
    const std::vector<double> times{0.1, 2.5, 5.0, 7.5, 10.0};
    for (PointId i = 0; i < times.size(); ++i)
    {
        view->setField(DimId::X, i, 0.0);
        view->setField(DimId::Y, i, 0.0);
        view->setField(DimId::Z, i, 0.0);
        view->setField(DimId::GpsTime, i, times[i]);
    }

    BufferReader reader;
    reader.addView(view);
    GeoreferenceFilter filter;
    Options options;
    options.add("trajectory_file", filename);
    options.add("scan2imu", identityTransform());
    filter.setOptions(options);
    filter.setInput(reader);

    ReprojectionFilter reprojection;
    Options reprojectionOptions;
    reprojectionOptions.add("in_srs", "EPSG:4978");
    reprojectionOptions.add("out_srs", "EPSG:4979");
    reprojection.setOptions(reprojectionOptions);
    reprojection.setInput(filter);
    reprojection.prepare(table);
    const PointViewSet views = reprojection.execute(table);
    ASSERT_EQ(views.size(), 1u);
    const PointViewPtr result = *views.begin();
    ASSERT_EQ(result->size(), times.size());
    for (PointId i = 0; i < times.size(); ++i)
    {
        SCOPED_TRACE(times[i]);
        EXPECT_NEAR(result->getFieldAs<double>(DimId::Z, i),
            50.0 + 10.0 * times[i], 1e-6);
    }
    FileUtils::deleteFile(filename);
}

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
    double ecef1X = p1.getFieldAs<double>(DimId::X);
    double ecef1Y = p1.getFieldAs<double>(DimId::Y);
    double ecef1Z = p1.getFieldAs<double>(DimId::Z);
    
    PointRef p2(*result, 1);
    double ecef2X = p2.getFieldAs<double>(DimId::X);
    double ecef2Y = p2.getFieldAs<double>(DimId::Y);
    double ecef2Z = p2.getFieldAs<double>(DimId::Z);
    
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
