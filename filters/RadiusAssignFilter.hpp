#pragma once

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>
#include <unordered_map>
#include "private/expr/AssignStatement.hpp"


extern "C" int32_t RadiusAssignFilter_ExitFunc();
extern "C" PF_ExitFunc RadiusAssignFilter_InitPlugin();

namespace pdal
{

struct DimRange;

class PDAL_EXPORT RadiusAssignFilter : public Filter
{
public:
    RadiusAssignFilter();
    ~RadiusAssignFilter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const { return "filters.radiusassign"; }

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void preparedDomain(std::vector<DimRange> &domain, PointLayoutPtr layout);
    virtual void prepared(PointTableRef table);
    bool doOne(PointRef& point);
    void doOneNoDomain(PointRef &point);
    virtual void filter(PointView& view);
    virtual void initializeDomain(StringList domainSpec, std::vector<DimRange> &domain);
    virtual void initialize();
    virtual void ready(PointTableRef);
    RadiusAssignFilter& operator=(
        const RadiusAssignFilter&) = delete;
    RadiusAssignFilter(const RadiusAssignFilter&) = delete;
    StringList m_referenceDomainSpec;
    std::vector<DimRange> m_referenceDomain;
    StringList m_srcDomainSpec;
    std::vector<DimRange> m_srcDomain;
    double m_radius;
    std::vector<expr::AssignStatement> m_updateExpr;
    bool m_search3d;
    double m_max2dAbove;
    double m_max2dBelow;

    PointViewPtr refView;
    PointIdList m_ptsToUpdate;
};

} // namespace pdal
