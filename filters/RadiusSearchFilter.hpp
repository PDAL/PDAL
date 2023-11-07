#pragma once

#include <pdal/Filter.hpp>
#include <pdal/KDIndex.hpp>
#include <unordered_map>

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
    bool doOne(PointRef& point, PointRef& temp, KD2Index &kdi);
    virtual void filter(PointView& view);
    virtual void initializeDomain(StringList domainSpec, std::vector<DimRange> &domain);
    virtual void initialize();
    virtual void ready(PointTableRef);
    void doOneNoDomain(PointRef &point, PointRef& temp, KD2Index &kdi);
    RadiusSearchFilter& operator=(
        const RadiusSearchFilter&) = delete;
    RadiusSearchFilter(const RadiusSearchFilter&) = delete;
    StringList m_referenceDomainSpec;
    std::vector<DimRange> m_referenceDomain;
    StringList m_srcDomainSpec;
    std::vector<DimRange> m_srcDomain;
    double m_radius;
    Dimension::Id m_outputDim;
    std::string m_outputDimName;
    double m_outputValue;
    std::unordered_map<PointId, double> m_newValue;

};

} // namespace pdal
