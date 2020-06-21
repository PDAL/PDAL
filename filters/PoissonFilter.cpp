/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "PoissonFilter.hpp"
#include "NormalFilter.hpp"
#include "TransformationFilter.hpp"

#include <kazhdan/PoissonRecon.h>
#include <kazhdan/point_source/PointSource.h>

// Note: For testing, download the eagle set here:
// https://www.cs.jhu.edu/~misha/Code/PoissonRecon/Version8.0/
// or here:
// https://drive.google.com/file/d/11dYMNmNx3XVh0m13OYjW4UX3Bhb4aNXg/view?usp=sharing
// Run with depth = 10.  You should get a good looking eagle out
// that you can view with meshlab.  The output should have 1,893,883 vertices
// and 3,787,635 faces.
namespace pdal
{

class PointViewSource : public PointSource
{
public:
    PointViewSource(PointView& view) : m_view(view), m_current(0)
        {}

    virtual void reset()
        { m_current = 0; }
    virtual bool nextPoint(Point& point)
    {
        using namespace Dimension;

        if (m_current >= m_view.size())
            return false;
        point.p.coords[0] = m_view.getFieldAs<double>(Id::X, m_current);
        point.p.coords[1] = m_view.getFieldAs<double>(Id::Y, m_current);
        point.p.coords[2] = m_view.getFieldAs<double>(Id::Z, m_current);
        point.n.coords[0] = m_view.getFieldAs<double>(Id::NormalX, m_current);
        point.n.coords[1] = m_view.getFieldAs<double>(Id::NormalY, m_current);
        point.n.coords[2] = m_view.getFieldAs<double>(Id::NormalZ, m_current);
        m_current++;
        return true;
    }

private:
    PointView& m_view;
    PointId m_current;
};

class ColorPointViewSource : public ColorPointSource
{
public:
    ColorPointViewSource(PointView& view) : m_view(view), m_current(0)
        {}

    virtual void reset()
        { m_current = 0; }
    virtual bool nextPoint(Point& point, Point3D<double>& color)
    {
        using namespace Dimension;

        if (m_current >= m_view.size())
            return false;
        point.p.coords[0] = m_view.getFieldAs<double>(Id::X, m_current);
        point.p.coords[1] = m_view.getFieldAs<double>(Id::Y, m_current);
        point.p.coords[2] = m_view.getFieldAs<double>(Id::Z, m_current);
        point.n.coords[0] = m_view.getFieldAs<double>(Id::NormalX, m_current);
        point.n.coords[1] = m_view.getFieldAs<double>(Id::NormalY, m_current);
        point.n.coords[2] = m_view.getFieldAs<double>(Id::NormalZ, m_current);
        color.coords[0] = m_view.getFieldAs<double>(Id::Red, m_current);
        color.coords[1] = m_view.getFieldAs<double>(Id::Green, m_current);
        color.coords[2] = m_view.getFieldAs<double>(Id::Blue, m_current);
        m_current++;
        return true;
    }

private:
    PointView& m_view;
    PointId m_current;
};

class PointViewMesh : public Kazhdan::Mesh
{
public:
    PointViewMesh(PointView& view, bool color) :
        m_view(view), m_mesh(*(m_view.createMesh("poisson"))), m_doColor(color)
    {
        resetIterator();
    }

    virtual int pointCount() const
        { return static_cast<int>(m_view.size()); }
    virtual int polygonCount() const
        { return m_mesh.size(); }
    virtual int newPoint(const std::array<double, 3>& position)
    {
        PointId cnt = m_view.size();
        m_view.setField(Dimension::Id::X, cnt, position[0]);
        m_view.setField(Dimension::Id::Y, cnt, position[1]);
        m_view.setField(Dimension::Id::Z, cnt, position[2]);
        return static_cast<int>(cnt);
    }

    virtual int newPoint(const std::array<double, 3>& position, double density)
    {
        PointId cnt = m_view.size();
        m_view.setField(Dimension::Id::X, cnt, position[0]);
        m_view.setField(Dimension::Id::Y, cnt, position[1]);
        m_view.setField(Dimension::Id::Z, cnt, position[2]);
        m_view.setField(Dimension::Id::Density, cnt, density);
        return static_cast<int>(cnt);
    }

    virtual int newPoint(const std::array<double, 3>& position,
        const std::array<uint8_t, 3>& color)
    {
        PointId cnt = m_view.size();
        m_view.setField(Dimension::Id::X, cnt, position[0]);
        m_view.setField(Dimension::Id::Y, cnt, position[1]);
        m_view.setField(Dimension::Id::Z, cnt, position[2]);
        m_view.setField(Dimension::Id::Red, cnt, color[0]);
        m_view.setField(Dimension::Id::Green, cnt, color[1]);
        m_view.setField(Dimension::Id::Blue, cnt, color[2]);
        return static_cast<int>(cnt);
    }

    virtual int newPoint(const std::array<double, 3>& position,
        const std::array<uint8_t, 3>& color, double density)
    {
        PointId cnt = m_view.size();
        m_view.setField(Dimension::Id::X, cnt, position[0]);
        m_view.setField(Dimension::Id::Y, cnt, position[1]);
        m_view.setField(Dimension::Id::Z, cnt, position[2]);
        m_view.setField(Dimension::Id::Red, cnt, color[0]);
        m_view.setField(Dimension::Id::Green, cnt, color[1]);
        m_view.setField(Dimension::Id::Blue, cnt, color[2]);
        m_view.setField(Dimension::Id::Density, cnt, density);
        return static_cast<int>(cnt);
    }

    virtual void newPolygon(std::vector<int>& poly)
    {
        assert(poly.size() == 3);
        m_mesh.add(poly[0], poly[1], poly[2]);
    }

    virtual bool hasDensity() const
        { return m_view.hasDim(Dimension::Id::Density); }

    virtual void resetIterator()
    {
        m_polyIdx = 0;
        m_pointIdx = 0;
    }

    virtual bool nextPolygon(Kazhdan::Polygon& poly)
    {
        if (m_polyIdx >= m_mesh.size())
            return false;

        const Triangle& t = m_mesh[m_polyIdx];
        poly.insert(poly.end(), { (int)t.m_a, (int)t.m_b, (int)t.m_c } );
        m_polyIdx++;
        return true;
    }

    virtual bool nextPoint(Kazhdan::Point& point)
    {
        if (m_pointIdx > m_view.size())
            return false;
        point.m_position[0] = m_view.getFieldAs<double>(Dimension::Id::X,
            m_pointIdx);
        point.m_position[1] = m_view.getFieldAs<double>(Dimension::Id::Y,
            m_pointIdx);
        point.m_position[2] = m_view.getFieldAs<double>(Dimension::Id::Z,
            m_pointIdx);
        point.m_density = m_view.getFieldAs<double>(Dimension::Id::Density,
            m_pointIdx);
        if (m_doColor)
        {
            point.m_color[0] = m_view.getFieldAs<uint8_t>(Dimension::Id::Red,
                m_pointIdx);
            point.m_color[1] = m_view.getFieldAs<uint8_t>(Dimension::Id::Green,
                m_pointIdx);
            point.m_color[2] = m_view.getFieldAs<uint8_t>(Dimension::Id::Blue,
                m_pointIdx);
        }
        m_pointIdx++;
        return true;
    }

private:
    PointView& m_view;
    TriangularMesh& m_mesh;
    size_t m_polyIdx;
    size_t m_pointIdx;
    bool m_doColor;
};

static StaticPluginInfo const s_info
{
    "filters.poisson",
    "Poisson Surface Reconstruction Filter",
    "http://pdal.io/stages/filters.poisson.html"
};

CREATE_STATIC_STAGE(PoissonFilter, s_info)

std::string PoissonFilter::getName() const { return s_info.name; }

void PoissonFilter::addDimensions(PointLayoutPtr layout)
{
    if (layout->hasDim(Dimension::Id::Red) &&
        layout->hasDim(Dimension::Id::Green) &&
        layout->hasDim(Dimension::Id::Blue))
        m_doColor = true;

    if (layout->hasDim(Dimension::Id::NormalX))
    {
        if ((!layout->hasDim(Dimension::Id::NormalY)) ||
            (!layout->hasDim(Dimension::Id::NormalZ)))
            throwError("If normals are provided as part of the input "
                "dataset, all of X, Y and Z must be provided.");
        m_normalsProvided = true;
    }
    else
        layout->registerDims( {Dimension::Id::NormalX, Dimension::Id::NormalY,
             Dimension::Id::NormalZ} );
}


void PoissonFilter::addArgs(ProgramArgs& args)
{
    args.add("density", "Output density estimates", m_density);
    args.add("depth", "Maximum depth of the octree used for reconstruction",
        m_depth, 8);
}


PointViewSet PoissonFilter::run(PointViewPtr view)
{
    if (!m_normalsProvided)
    {
        NormalFilter f;
        f.setLog(log());
        f.doFilter(*view);
    }

    std::unique_ptr<PointSource> source;
    if (m_doColor)
        source.reset(new ColorPointViewSource(*view));
    else
        source.reset(new PointViewSource(*view));

    PoissonOpts<double> opts;

    opts.m_depth = m_depth;
    opts.m_density = m_density;
    opts.m_solveDepth = m_depth;
    opts.m_kernelDepth = m_depth - 2;
    if (m_doColor)
    {
        opts.m_color = 16;
        opts.m_hasColor = true;
    }

    PoissonRecon<double> recon(opts, *source);
    if (!recon.execute())
        throwError("Failure executing poisson algorithm.");
    recon.evaluate();

    PointViewSet s;
    PointViewPtr outView = view->makeNew();
    s.insert(outView);
    PointViewMesh mesh(*outView, m_doColor);
    recon.extractMesh(mesh);

    // Note here that we're transforming the matrix in the traditional
    // sense (rows become columns) because the transformation filter does
    // right multiplication instead of left.
    TransformationFilter::Transform transform;
    auto xform = recon.inverseTransform();
    size_t i = 0;
    for (size_t c = 0; c < 4; ++c)
        for (size_t r = 0; r < 4; ++r)
            transform[i++] = xform.coords[r][c];

    TransformationFilter().doFilter(*outView, transform);
    return s;
}

} // namespace pdal
