#include "RadiusSearchFilter.hpp"

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"

#include <iostream>
#include <utility>
namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.radiussearch",
    "Re-assign some point attributes based KNN voting",
    "http://pdal.io/stages/filters.radiussearch.html" );

CREATE_STATIC_STAGE(RadiusSearchFilter, s_info)

RadiusSearchFilter::RadiusSearchFilter()
{}


RadiusSearchFilter::~RadiusSearchFilter()
{}


void RadiusSearchFilter::addArgs(ProgramArgs& args)
{
    args.add("src_domain", "Selects which points will be subject to "
        "radius-based neighbors search", m_srcDomainSpec);
    args.add("reference_domain", "Selects which points will be considered as "
        "potential neighbors", m_referenceDomainSpec);
    args.add("radius", "Distance of neighbors to consult",
        m_radius);
    args.add("output_dim", "Dimension for which to assign a value when at least one neighbor is found",
        m_outputDimName);
    args.add("output_value", "Value to assign when at least one neighbor is found",
        m_outputValue);
}


void RadiusSearchFilter::initializeDomain(StringList domainSpec, std::vector<DimRange> &domain)
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

void RadiusSearchFilter::initialize()
{
    this->initializeDomain(m_referenceDomainSpec, m_referenceDomain);
    this->initializeDomain(m_srcDomainSpec, m_srcDomain);

    if (m_radius <= 0)
        throwError("Invalid 'radius' option: " + std::to_string(m_radius) +
            ", must be > 0");
}

void RadiusSearchFilter::preparedDomain(std::vector<DimRange> &domain, PointLayoutPtr layout)
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

void RadiusSearchFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());
    this->preparedDomain(m_srcDomain, layout);
    this->preparedDomain(m_referenceDomain, layout);

    m_outputDim = layout->findDim(m_outputDimName);
    if (m_outputDim == Dimension::Id::Unknown)
        throwError("Invalid dimension name '" + m_outputDimName + "'.");
}


void RadiusSearchFilter::ready(PointTableRef)
{
    m_newValue.clear();
}


void RadiusSearchFilter::doOneNoDomain(PointRef &point, PointRef &temp,
    KD2Index &kdi)
{
    PointIdList iNeighbors = kdi.radius(point, m_radius);
    if (iNeighbors.size() == 0)
        return;

    m_newValue[point.pointId()] = m_outputValue;
}

// update point.  kdi and temp both reference the NN point cloud
bool RadiusSearchFilter::doOne(PointRef& point, PointRef &temp,
    KD2Index &kdi)
{
    if (m_srcDomain.empty())  // No domain, process all points
        doOneNoDomain(point, temp, kdi);

    for (DimRange& r : m_srcDomain)
    {   // process only points that satisfy a domain condition
        if (r.valuePasses(point.getFieldAs<double>(r.m_id)))
        {
            doOneNoDomain(point, temp, kdi);
            break;
        }
    }
    return true;
}

void RadiusSearchFilter::filter(PointView& view)
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
    PointRef point_nn(*refView, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point_src.setPointId(id);
        doOne(point_src, point_nn, kdiRef);
    }

    for (auto& p : m_newValue)
        view.setField(m_outputDim, p.first, p.second);
}

} // namespace pdal

