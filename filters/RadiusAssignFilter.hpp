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
    struct Private;

public:
    RadiusAssignFilter();
    ~RadiusAssignFilter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const { return "filters.radiusassign"; }

private:
    RadiusAssignFilter& operator=(const RadiusAssignFilter&) = delete;
    RadiusAssignFilter(const RadiusAssignFilter&) = delete;

    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    bool doOne(PointRef& point);
    void doOneNoDomain(PointRef &point);
    virtual void filter(PointView& view);
    virtual void initialize();
    virtual void ready(PointTableRef);

    std::unique_ptr<Private> m_p;
};

} // namespace pdal
