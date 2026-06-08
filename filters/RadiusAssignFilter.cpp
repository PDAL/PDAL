#include "RadiusAssignFilter.hpp"

#include <vector>

#include <pdal/KDIndex.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"
#include "private/expr/AssignStatement.hpp"

#include <utility>
namespace pdal
{

struct RadiusAssignFilter::Private
{
    DimRangeList referenceDomain;
    DimRangeList srcDomain;
    double radius;
    std::vector<expr::AssignStatement> updateExpr;
    bool search3d;
    double max2dAbove;
    double max2dBelow;
    PointViewPtr refView;
    PointIdList ptsToUpdate;
};

static PluginInfo const RadiusAssignFilter_info = PluginInfo(
    "filters.radiusassign",
    "Re-assign some point attributes based KNN voting",
    "https://pdal.org/stages/filters.radiusassign.html" );

CREATE_STATIC_STAGE(RadiusAssignFilter, RadiusAssignFilter_info)

RadiusAssignFilter::RadiusAssignFilter() : m_p(new Private)
{}


RadiusAssignFilter::~RadiusAssignFilter()
{}


void RadiusAssignFilter::addArgs(ProgramArgs& args)
{
    args.add("src_domain", "Selects which points will be subject to "
        "radius-based neighbors search", m_p->srcDomain);
    args.add("reference_domain", "Selects which points will be considered as "
        "potential neighbors", m_p->referenceDomain);
    args.add("radius", "Distance of neighbors to consult", m_p->radius);
    args.add("update_expression", "Value to assign to dimension of points of src_domain "
            "that have at least one neighbor in reference domain based on expression.",
        m_p->updateExpr);
    args.add("is3d", "Search in 3d", m_p->search3d, false);
    args.add("max2d_above", "if search in 2d : upward maximum distance in Z for "
        "potential neighbors (corresponds to a search in a cylinder with a height = "
        "max2d_above above the source point). Values < 0 mean infinite height",
        m_p->max2dAbove, -1.);
    args.add("max2d_below", "if search in 2d : downward maximum distance in Z "
        "for potential neighbors (corresponds to a search in a cylinder with a height = "
        "max2d_below below the source point). Values < 0 mean infinite height",
        m_p->max2dBelow, -1.);
}

void RadiusAssignFilter::initialize()
{
    if (m_p->radius <= 0)
        throwError("Invalid 'radius' option: " + std::to_string(m_p->radius) + ", must be > 0");
    if (m_p->updateExpr.size() == 0)
        throwError("Empty 'update_epxression' option, must be set to apply any change on the data");
}


void RadiusAssignFilter::prepared(PointTableRef table)
{
    Utils::StatusWithReason status;

    PointLayoutPtr layout(table.layout());
    status = m_p->srcDomain.prepare(layout);
    if (!status)
        throwError("Invalid dimension name in 'src_domain': " + status.what());
    status = m_p->referenceDomain.prepare(layout);
    if (!status)
        throwError("Invalid dimension name in 'reference_domain': " + status.what());

    for (expr::AssignStatement& expr : m_p->updateExpr)
    {
        status = expr.prepare(layout);
        if (!status)
            throwError("Invalid assignment expression in 'update_expression' option: " +
                status.what());
    }
}


void RadiusAssignFilter::ready(PointTableRef)
{
    m_p->ptsToUpdate.clear();
}


void RadiusAssignFilter::doOneNoDomain(PointRef &pointSrc)
{
    PointIdList iNeighbors;
    if (m_p->search3d)
        iNeighbors = m_p->refView->build3dIndex().radius(pointSrc, m_p->radius);
    else
        iNeighbors = m_p->refView->build2dIndex().radius(pointSrc, m_p->radius);

    if (iNeighbors.size() == 0)
        return;

    if (!m_p->search3d && (m_p->max2dBelow>=0 || m_p->max2dAbove>=0))
    {
        double Zsrc = pointSrc.getFieldAs<double>(Dimension::Id::Z);

        bool take (false);
        for (PointId ptId : iNeighbors)
        {
            double Zref = m_p->refView->point(ptId).getFieldAs<double>(Dimension::Id::Z);
            if (m_p->max2dAbove>=0 && Zref>Zsrc && (Zref-Zsrc)>m_p->max2dAbove) continue;
            if (m_p->max2dBelow>=0 && Zsrc>Zref && (Zsrc-Zref)>m_p->max2dBelow) continue;
            take = true;
            break;
        }

        if (!take)
            return;
    }

    m_p->ptsToUpdate.push_back(pointSrc.pointId());
}

// update point.  kdi and temp both reference the NN point cloud
bool RadiusAssignFilter::doOne(PointRef& pointSrc)
{
    if (m_p->srcDomain.empty())  // No domain, process all points
        doOneNoDomain(pointSrc);

    for (const DimRange& r : m_p->srcDomain.ranges())
    {   // process only points that satisfy a domain condition
        if (r.valuePasses(pointSrc.getFieldAs<double>(r.m_id)))
        {
            doOneNoDomain(pointSrc);
            break;
        }
    }
    return true;
}

void RadiusAssignFilter::filter(PointView& view)
{
    PointRef pointTemp(view, 0);

    // Create reference domain view
    m_p->refView = view.makeNew();
    if (m_p->referenceDomain.empty())
        for (PointId id = 0; id < view.size(); ++id)
            m_p->refView->appendPoint(view, id);
    else
    {
        for (PointId id = 0; id < view.size(); ++id)
        {
            for (const DimRange& r : m_p->referenceDomain.ranges())
            {
                pointTemp.setPointId(id);
                if (r.valuePasses(pointTemp.getFieldAs<double>(r.m_id)))
                {
                    m_p->refView->appendPoint(view, id);
                    break;
                }
            }
        }
    }

    // Process all points (mark them if they need to be updated)
    for (PointId id = 0; id < view.size(); ++id)
    {
        pointTemp.setPointId(id);
        doOne(pointTemp);
    }

    for (auto id: m_p->ptsToUpdate)
    {
        pointTemp.setPointId(id);
        for (expr::AssignStatement& expr : m_p->updateExpr)
            if (expr.conditionalExpr().eval(pointTemp))
                pointTemp.setField(expr.identExpr().eval(), expr.valueExpr().eval(pointTemp));
    }
}

} // namespace pdal
