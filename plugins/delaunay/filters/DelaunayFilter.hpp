// TODO: insert copyright notice

#pragma once

#include <pdal/Filter.hpp>

namespace pdal
{

class PDAL_DLL DelaunayFilter : public Filter
{
public:
    DelaunayFilter() : Filter()
    {}
    std::string getName() const;

private:
    virtual PointViewSet run(PointViewPtr view);

    DelaunayFilter& operator=(const DelaunayFilter&); // not implemented
    DelaunayFilter(const DelaunayFilter&); // not implemented
};

} // namespace pdal
