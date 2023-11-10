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


void RadiusAssignFilter::doOneNoDomain(PointRef &point, KD2Index &kdi)
{
    PointIdList iNeighbors = kdi.radius(point, m_radius);
    if (iNeighbors.size() == 0)
        return;

    m_ptsToUpdate.push_back(point.pointId());

}

// update point.  kdi and temp both reference the NN point cloud
bool RadiusAssignFilter::doOne(PointRef& point, KD2Index &kdi)
{
    if (m_srcDomain.empty())  // No domain, process all points
        doOneNoDomain(point, kdi);

    for (DimRange& r : m_srcDomain)
    {   // process only points that satisfy a domain condition
        if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
        {
            doOneNoDomain(point, kdi);
            break;
        }
    }
    return true;
}

void RadiusAssignFilter::filter(PointView& view)
{
    PointRef point_src(view, 0);
    // Create a kd tree only with the points in the reference domain (to make the search faster)
    PointViewPtr refView;
    PointRef temp(view, 0);
    if (m_referenceDomain.empty())
        for (PointId id = 0; id < view.size(); ++id)
            refView->appendPoint(view, id);
    else
    {
        refView = view.makeNew();
        for (PointId id = 0; id < view.size(); ++id)
        {
            for (DimRange& r : m_referenceDomain)
            {   // process only points that satisfy a domain condition
                temp.setPointId(id);
                if (r.valuePasses(temp.getFieldAs<double>(r.m_id)))
                {
                    refView->appendPoint(view, id);
                    break;
                }
            }
        }
    }

    KD2Index& kdiRef = refView->build2dIndex();
    for (PointId id = 0; id < view.size(); ++id)
    {
        point_src.setPointId(id);
        doOne(point_src, kdiRef);
    }
    for (auto id: m_ptsToUpdate)
    {
        temp.setPointId(id);
        for (expr::AssignStatement& expr : m_updateExpr)
            // if (expr.conditionalExpr().eval(temp))
                temp.setField(expr.identExpr().eval(), expr.valueExpr().eval(temp));
    }
}

} // namespace pdal

