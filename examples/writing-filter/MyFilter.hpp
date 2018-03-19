// MyFilter.hpp

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Stage.hpp>

#include <memory>

namespace pdal
{

class Options;
class PointLayout;
class PointView;

class PDAL_DLL MyFilter : public Filter
{
public:
    MyFilter() : Filter()
    {}
    std::string getName() const;

private:
    double m_value;
    Dimension::Id m_myDimension;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void addArgs(ProgramArgs& args);
    virtual PointViewSet run(PointViewPtr view);

    MyFilter& operator=(const MyFilter&); // not implemented
    MyFilter(const MyFilter&); // not implemented
};

} // namespace pdal
