/******************************************************************************
* Copyright (c) 2015, Bradley J Chambers, brad.chambers@gmail.com
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

#pragma once

#include <pdal/Writer.hpp>

#include <Eigen/Core>

#include <string>

#include "gdal_priv.h" // For File I/O

extern "C" int32_t DerivativeWriter_ExitFunc();
extern "C" PF_ExitFunc DerivativeWriter_InitPlugin();

namespace pdal
{

class BOX2D;

class PDAL_DLL DerivativeWriter : public Writer
{
    enum SlopeMethod
    {
        SD8,
        SFD
    };

    enum AspectMethod
    {
        AD8,
        AFD
    };

    enum PrimitiveType
    {
        SLOPE_D8,
        SLOPE_FD,
        ASPECT_D8,
        ASPECT_FD,
        HILLSHADE,
        CONTOUR_CURVATURE,
        PROFILE_CURVATURE,
        TANGENTIAL_CURVATURE,
        TOTAL_CURVATURE,
        CATCHMENT_AREA
    };

    enum CurvatureType
    {
        CONTOUR,
        PROFILE,
        TANGENTIAL,
        TOTAL
    };

    enum Direction
    {
        NORTH,
        SOUTH,
        EAST,
        WEST,
        NORTHEAST,
        NORTHWEST,
        SOUTHEAST,
        SOUTHWEST
    };

public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    DerivativeWriter();

    Options getDefaultOptions();

private:
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr view);

    void setBounds(const BOX2D& v)
    {
        m_bounds = v;
    }

    BOX2D& getBounds()
    {
        return m_bounds;
    }

    void calculateGridSizes();
    double determineSlopeFD(Eigen::MatrixXd* data, int row, int col,
                            double postSpacing, double valueToIgnore);
    double determineSlopeD8(Eigen::MatrixXd* data, int row, int col,
                            double postSpacing, double valueToIgnore);
    double determineAspectFD(Eigen::MatrixXd* data, int row, int col,
                             double postSpacing, double valueToIgnore);
    double determineAspectD8(Eigen::MatrixXd* data, int row, int col,
                             double postSpacing);
    int determineCatchmentAreaD8(Eigen::MatrixXd* data, Eigen::MatrixXd* area,
                                 int row, int col, double postSpacing);
    double determineContourCurvature(Eigen::MatrixXd* data, int row, int col,
                                     double postSpacing, double valueToIgnore);
    double determineProfileCurvature(Eigen::MatrixXd* data, int row, int col,
                                     double postSpacing, double valueToIgnore);
    double determineTangentialCurvature(Eigen::MatrixXd* data, int row, int col,
                                        double postSpacing, double valueToIgnore);
    double determineTotalCurvature(Eigen::MatrixXd* data, int row, int col,
                                   double postSpacing, double valueToIgnore);
    double determineHillshade(Eigen::MatrixXd* data, int row, int col,
                              double zenithRad, double azimuthRad,
                              double postSpacing);
    double GetNeighbor(Eigen::MatrixXd* data, int row, int col, Direction d);
    void writeSlope(Eigen::MatrixXd* dem, const PointViewPtr cloud,
                    SlopeMethod method=SD8);
    void writeAspect(Eigen::MatrixXd* dem, const PointViewPtr cloud,
                     AspectMethod method=AD8);
    void writeCatchmentArea(Eigen::MatrixXd* dem, const PointViewPtr cloud);
    void writeHillshade(Eigen::MatrixXd* dem, const PointViewPtr cloud);
    void writeCurvature(Eigen::MatrixXd* dem, const PointViewPtr cloud,
                        CurvatureType curveType, double valueToIgnore);
    GDALDataset* createFloat32GTIFF(std::string filename, int cols, int rows);
    void stretchData(float *data);

    boost::uint64_t m_pointCount;
    boost::uint32_t m_GRID_SIZE_X;
    boost::uint32_t m_GRID_SIZE_Y;
    double m_GRID_DIST_X;
    double m_GRID_DIST_Y;
    unsigned int m_primitive_type;
    BOX2D m_bounds;
    std::string m_filename;
    SpatialReference m_inSRS;

    DerivativeWriter& operator=(const DerivativeWriter&); // not implemented
    DerivativeWriter(const DerivativeWriter&); // not implemented
};

} // namespace pdal
