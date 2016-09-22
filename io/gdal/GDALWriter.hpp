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

#include <pdal/PointView.hpp>
#include <pdal/Writer.hpp>
#include <pdal/plugin.hpp>

extern "C" int32_t GDALWriter_ExitFunc();
extern "C" PF_ExitFunc GDALWriter_InitPlugin();

namespace pdal
{

class Grid
{
public:
    Grid(size_t width, size_t height, double edgeLength, double radius,
            double noData) :
        m_width(width), m_height(height), m_edgeLength(edgeLength),
        m_radius(radius), m_noData(noData),
        m_count(width * height),
        m_min(width * height, std::numeric_limits<double>::max()),
        m_max(width * height, std::numeric_limits<double>::lowest()),
        m_mean(width * height), m_stdDev(width * height),
        m_idw(width * height), m_idwDist(width * height)
    {}

    int horizontalIndex(double x)
        { return x / m_edgeLength; }

    int verticalIndex(double y)
        { return m_height - (y / m_edgeLength) - 1; }

    double horizontalPos(size_t i)
        { return (i + .5) * m_edgeLength; }

    double verticalPos(size_t j)
        { return (m_height - (j + .5)) * m_edgeLength; }

    double distance(size_t i, size_t j, double x, double y)
    {
        double x1 = horizontalPos(i);
        double y1 = verticalPos(j);
        return sqrt(pow(x1 - x, 2) + pow(y1 - y, 2));
    }

    int numBands() const
        { return 6; }

    uint8_t *data(const std::string& name)
    {
        if (name == "count")
            return (uint8_t *)m_count.data();
        if (name == "min")
            return (uint8_t *)m_min.data();
        if (name == "max")
            return (uint8_t *)m_max.data();
        if (name == "mean")
            return (uint8_t *)m_mean.data();
        if (name == "idw")
            return (uint8_t *)m_idw.data();
        if (name == "stdev")
            return (uint8_t *)m_stdDev.data();
        throw pdal_error("Requested invalid grid data '" + name + "'.");
    }

    void addPoint(double x, double y, double z)
    {
        int iOrigin = horizontalIndex(x);
        int jOrigin = verticalIndex(y);

        // First quadrant;
        int i = iOrigin + 1;
        int j = jOrigin;
        while (i < (int)m_width && j >= 0)
        {
            double d = distance(i, j, x, y);
            if (d < m_radius)
            {
                update(i, j, z, d);
                i++;
            }
            else
            {
                if (i == iOrigin + 1)
                    break;
                i = iOrigin + 1;
                j--;
            }
        }

        // Second quadrant;
        i = iOrigin;
        j = jOrigin - 1;
        while (i >= 0 && j >= 0)
        {
            double d = distance(i, j, x, y);
            if (d < m_radius)
            {
                update(i, j, z, d);
                j--;
            }
            else
            {
                if (j == jOrigin - 1)
                    break;
                j = jOrigin - 1;
                i--;
            }
        }

        // Third quadrant;
        i = iOrigin - 1;
        j = jOrigin;
        while (i >= 0 && j < (int)m_height)
        {
            double d = distance(i, j, x, y);
            if (d < m_radius)
            {
                update(i, j, z, d);
                i--;
            }
            else
            {
                if (i == iOrigin - 1)
                    break;
                i = iOrigin - 1;
                j++;
            }
        }

        // Fourth quadrant;
        i = iOrigin;
        j = jOrigin + 1;
        while (i < (int)m_width && j < (int)m_height)
        {
            double d = distance(i, j, x, y);
            if (d < m_radius)
            {
                update(i, j, z, d);
                j++;
            }
            else
            {
                if (j == jOrigin + 1)
                    break;
                j = jOrigin + 1;
                i++;
            }
        }

        // This is a questionable case.  If a point is in a cell, shouldn't
        // it just be counted?
        double d = distance(iOrigin, jOrigin, x, y);
        if (d < m_radius)
            update(iOrigin, jOrigin, z, d);
    }

    void update(int x, int y, double val, double dist)
    {
        // See
        // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        // https://en.wikipedia.org/wiki/Inverse_distance_weighting

        size_t offset = (y * m_width) + x;

        double& count = m_count[offset];
        count++;

        double& min = m_min[offset];
        min = std::min(val, min);

        double& max = m_max[offset];
        max = std::max(val, max);

        double& mean = m_mean[offset];
        double delta = val - mean;

        mean += delta / count;

        double& stdDev = m_stdDev[offset];
        stdDev += delta * (val - mean);

        double& idw = m_idw[offset];
        idw += val / dist;

        double& idwDist = m_idwDist[offset];
        idwDist += 1 / dist; 
    }

    void finalize()
    {
        // See
        // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
        // https://en.wikipedia.org/wiki/Inverse_distance_weighting
        for (size_t i = 0; i < m_count.size(); ++i)
        {
            if (m_count[i])
            {
                m_stdDev[i] = sqrt(m_stdDev[i] / m_count[i]);
                m_idw[i] /= m_idwDist[i]; 
            }
            else
            {
                m_min[i] = m_noData;
                m_max[i] = m_noData;
            }
        }
    }

    size_t width() const
        { return m_width; }
    size_t height() const
        { return m_height; }
    double noData() const
        { return m_noData; }

private:
    size_t m_width;
    size_t m_height;
    double m_edgeLength;
    double m_radius;
    double m_noData;
    std::vector<double> m_count;
    std::vector<double> m_min;
    std::vector<double> m_max;
    std::vector<double> m_mean;
    std::vector<double> m_stdDev;
    std::vector<double> m_idw;
    std::vector<double> m_idwDist;
};
typedef std::unique_ptr<Grid> GridPtr;

class PDAL_DLL GDALWriter : public Writer
{
public:
    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void ready(PointTableRef table);
    virtual void write(const PointViewPtr data);
    virtual void done(PointTableRef table);

    std::string m_drivername;
    BOX2D m_bounds;
    double m_edgeLength;
    double m_radius;
    StringList m_options;
    GridPtr m_grid;
};

}
