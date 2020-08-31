/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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

#include <math.h>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include <pdal/pdal_internal.hpp>
#include <pdal/private/Raster.hpp>

namespace pdal
{

class GDALGrid
{
    FRIEND_TEST(GDALWriterTest, issue_2095);
public:
    static const int statCount = 1;
    static const int statMin = 2;
    static const int statMax = 4;
    static const int statMean = 8;
    static const int statStdDev = 16;
    static const int statIdw = 32;

    struct error : public std::runtime_error
    {
        error(const std::string& err) : std::runtime_error(err)
        {}
    };

    // Exported for testing.
    PDAL_DLL GDALGrid(double xOrigin, double yOrigin, size_t width, size_t height,
        double edgeLength, double radius, int outputTypes, size_t windowSize,
        double power);

    void expandToInclude(double x, double y);

    // Get the number of bands represented by this grid.
    int numBands() const;

    // Return a pointer to the data in a raster band, row-major ordered.
    double *data(const std::string& name);

    // Add a point to the raster grid.
    void addPoint(double x, double y, double z);

    // Compute final values after all points have been added.
    void finalize();

    int width() const;
    int height() const;
    double xOrigin() const;
    double yOrigin() const;

private:
    int m_windowSize;
    double m_edgeLength;
    double m_radius;
    double m_power;

    typedef std::unique_ptr<Rasterd> DataPtr;
    DataPtr m_count;
    DataPtr m_min;
    DataPtr m_max;
    DataPtr m_mean;
    DataPtr m_stdDev;
    DataPtr m_idw;
    DataPtr m_idwDist;

    int m_outputTypes;

    // Determine if a cell i, j has no associated points.
    bool empty(size_t i, size_t j) const
        { return m_count->at(i, j) <= 0; }

    // Determine if a cell with index \c idx has no associated points.
    bool empty(size_t idx) const
        { return (m_count->at(idx) <= 0); }

    // Determine the distance from the center of cell at coordinate i, j to
    // a point at absolute coordinate x, y.
    double distance(int i, int j, double x, double y) const;

    // Update cells in the Nth quadrant about point at (x, y, z)
    void updateFirstQuadrant(double x, double y, double z);
    void updateSecondQuadrant(double x, double y, double z);
    void updateThirdQuadrant(double x, double y, double z);
    void updateFourthQuadrant(double x, double y, double z);

    // Update cell at i, j with value at a distance.
    void update(size_t i, size_t j, double val, double dist);

    // Fill cell at index \c i with the nondata value.
    // \i  I coordinate.
    // \j  J coordinate.
    void fillNodata(int i, int j);

    // Fill an empty cell with a value inverse-distance averaged from
    // surrounding cells.
    void windowFill();

    // Fill empty cell at dstI, dstJ with inverse-distance weighted values
    // from neighboring cells.
    void windowFill(int dstI, int dstJ);

    // Cumulate data from a source cell to a destination cell when doing
    // a window fill.
    void windowFillCell(int srcI, int srcJ, int dstI, int dstJ, double distance);
};

} //namespace pdal
