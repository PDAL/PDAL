#include "RadiusAssignFilter.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

#include <iostream>
#include <utility>
namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.radiusassign",
    "Re-assign some point attributes based KNN voting",
    "http://pdal.io/stages/filters.radiusassign.html" );

CREATE_STATIC_STAGE(RadiusAssignFilter, s_info)

RadiusAssignFilter::RadiusAssignFilter()
{}


RadiusAssignFilter::~RadiusAssignFilter()
{}


void RadiusAssignFilter::addArgs(ProgramArgs& args)
{
    args.add("src_domain", "Selects which points will be subject to "
        "radius-based neighbors search", m_srcDomainSpec);
    args.add("reference_domain", "Selects which points will be considered as "
        "potential neighbors", m_referenceDomainSpec);
    args.add("radius", "Distance of neighbors to consult",
        m_radius);
    args.add("update_expression", "Value to assign to dimension of points of src_domain "
            "that have at least one neighbor in reference domain based on expression.",
        m_updateExpr);
    args.add("is3d", "Search in 3d", m_search3d, false );
    args.add("max2d_above", "if search in 2d : upward maximum distance in Z for potential neighbors "
        "(corresponds to a search in a cylinder with a height = max2d_above above the source point). "
        "Values < 0 mean infinite height", m_max2dAbove, -1.);
    args.add("max2d_below", "if search in 2d : downward maximum distance in Z for potential neighbors ("
        "corresponds to a search in a cylinder with a height = max2d_below below the source point). "
        "Values < 0 mean infinite height", m_max2dBelow, -1.);

}


void RadiusAssignFilter::initializeDomain(StringList domainSpec, std::vector<DimRange> &domain)
{
    for (auto const& r : domainSpec)
    {
        try
        {
            DimRange range;
            range.parse(r);
            domain.push_back(range);
        }
        catch (const DimRange::error& err)
        {
            throwError("Invalid 'domain' option: '" + r + "': " + err.what());
        }
    }
}

void RadiusAssignFilter::initialize()
{
    this->initializeDomain(m_referenceDomainSpec, m_referenceDomain);
    this->initializeDomain(m_srcDomainSpec, m_srcDomain);

    if (m_radius <= 0)
        throwError("Invalid 'radius' option: " + std::to_string(m_radius) +
            ", must be > 0");
    if (m_updateExpr.size() == 0)
        throwError("Empty 'update_epxression' option, must be set to apply any change on the data");

}

void RadiusAssignFilter::preparedDomain(std::vector<DimRange> &domain, PointLayoutPtr layout)
{
    for (auto& r : domain)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Dimension::Id::Unknown)
            throwError("Invalid dimension name in 'srcDomain' option: '" +
                r.m_name + "'.");
    }
    std::sort(domain.begin(), domain.end());
}

void RadiusAssignFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    this->preparedDomain(m_srcDomain, layout);
    this->preparedDomain(m_referenceDomain, layout);


    for (expr::AssignStatement& expr : m_updateExpr)
    {
        auto status = expr.prepare(layout);
        if (!status)
            throwError("Invalid assignment expression in 'update_expression' option: " +
            status.what());
    }
}

void RadiusAssignFilter::ready(PointTableRef)
{
    m_ptsToUpdate.clear();
}


void RadiusAssignFilter::doOneNoDomain(PointRef &pointSrc)
{
    PointIdList iNeighbors;
    if (m_search3d) iNeighbors = refView->build3dIndex().radius(pointSrc, m_radius);
    else iNeighbors = refView->build2dIndex().radius(pointSrc, m_radius);


    if (iNeighbors.size() == 0)
        return;

    if (!m_search3d && (m_max2dBelow>=0 || m_max2dAbove>=0))
    {
        double Zsrc = pointSrc.getFieldAs<double>(Dimension::Id::Z);

        bool take (false);
        for (PointId ptId : iNeighbors)
        {
            double Zref = refView->point(ptId).getFieldAs<double>(Dimension::Id::Z);
            if (m_max2dAbove>=0 && Zref>Zsrc && (Zref-Zsrc)>m_max2dAbove) continue;
            if (m_max2dBelow>=0 && Zsrc>Zref && (Zsrc-Zref)>m_max2dBelow) continue;
            take = true;
            break;
        }

        if (!take) return;
    }


    m_ptsToUpdate.push_back(pointSrc.pointId());

}

// update point.  kdi and temp both reference the NN point cloud
bool RadiusAssignFilter::doOne(PointRef& pointSrc)
{
    if (m_srcDomain.empty())  // No domain, process all points
        doOneNoDomain(pointSrc);

    for (DimRange& r : m_srcDomain)
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
    refView = view.makeNew();
    if (m_referenceDomain.empty())
        for (PointId id = 0; id < view.size(); ++id)
            refView->appendPoint(view, id);
    else
    {
        for (PointId id = 0; id < view.size(); ++id)
        {
            for (DimRange& r : m_referenceDomain)
            {
                pointTemp.setPointId(id);
                if (r.valuePasses(pointTemp.getFieldAs<double>(r.m_id)))
                {
                    refView->appendPoint(view, id);
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


    for (auto id: m_ptsToUpdate)
    {
        pointTemp.setPointId(id);
        for (expr::AssignStatement& expr : m_updateExpr)
            if (expr.conditionalExpr().eval(pointTemp))
                pointTemp.setField(expr.identExpr().eval(), expr.valueExpr().eval(pointTemp));
    }
}


} // namespace pdal

