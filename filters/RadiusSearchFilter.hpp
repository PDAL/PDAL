#pragma once

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>
#include <unordered_map>
#include "private/expr/AssignStatement.hpp"


extern "C" int32_t RadiusSearchFilter_ExitFunc();
extern "C" PF_ExitFunc RadiusSearchFilter_InitPlugin();

namespace pdal
{

struct DimRange;

class PDAL_DLL RadiusSearchFilter : public Filter
{
public:
    RadiusSearchFilter();
    ~RadiusSearchFilter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const { return "filters.RadiusSearch"; }

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void preparedDomain(std::vector<DimRange> &domain, PointLayoutPtr layout);
    virtual void prepared(PointTableRef table);
    bool doOne(PointRef& point, KD2Index &kdi);
    void doOneNoDomain(PointRef &point, KD2Index &kdi);
    virtual void filter(PointView& view);
    virtual void initializeDomain(StringList domainSpec, std::vector<DimRange> &domain);
    virtual void initialize();
    virtual void ready(PointTableRef);
    RadiusSearchFilter& operator=(
        const RadiusSearchFilter&) = delete;
    RadiusSearchFilter(const RadiusSearchFilter&) = delete;
    StringList m_referenceDomainSpec;
    std::vector<DimRange> m_referenceDomain;
    StringList m_srcDomainSpec;
    std::vector<DimRange> m_srcDomain;
    double m_radius;
    std::vector<expr::AssignStatement> m_updateExpr;
    PointIdList m_ptsToUpdate;
};

} // namespace pdal
