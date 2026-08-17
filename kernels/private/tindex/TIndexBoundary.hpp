#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/PointRef.hpp>
#include <pdal/PointView.hpp>
#include <filters/private/hexer/HexGrid.hpp>

namespace pdal
{
namespace tindex
{

class TindexBoundary : public Filter, public Streamable
{
public:
    TindexBoundary(int32_t density, double edgeLength, uint32_t sampleSize)
        : m_density(density), m_edgeLength(edgeLength),
        m_sampleSize(sampleSize)
    {}
    ~TindexBoundary()
    {}

    std::string getName() const
    { return "tindex-boundary"; }
    double height()
    { return m_grid->height(); }
    std::string toWKT()
    {
        std::ostringstream out;
        out.setf(std::ios_base::fixed, std::ios_base::floatfield);
        out.precision(10);
        m_grid->toWKT(out);
        return out.str();
    }
private:
    std::unique_ptr<hexer::HexGrid> m_grid;
    int32_t m_density;
    double m_edgeLength;
    uint32_t m_sampleSize;

    virtual void ready(PointTableRef table)
    {
        if (m_edgeLength == 0.0)
        {
            m_grid.reset(new hexer::HexGrid(m_density));
            m_grid->setSampleSize(m_sampleSize);
        }
        else
            m_grid.reset(new hexer::HexGrid(m_edgeLength * sqrt(3), m_density));
    }
    virtual void filter(PointView& view)
    {
        PointRef p(view, 0);

        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            p.setPointId(idx);
            processOne(p);
        }
    }
    virtual bool processOne(PointRef& point)
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        m_grid->addXY(x, y);
        return true;
    }
    virtual void spatialReferenceChanged(const SpatialReference& srs)
    { setSpatialReference(srs); }
    virtual void done(PointTableRef table)
    {
        try
        {
            m_grid->findShapes();
            m_grid->findParentPaths();
        }
        catch (hexer::hexer_error& e)
        {
            throwError(e.what());
            m_grid.reset(new hexer::HexGrid(m_density));
        }
    }
};

} // namespace tindex
} // namespace pdal