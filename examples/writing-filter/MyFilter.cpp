// MyFilter.cpp

#include "MyFilter.hpp"

#include <pdal/pdal_internal.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "filters.name",
    "My awesome filter",
    "http://link/to/documentation"
};

CREATE_SHARED_STAGE(MyFilter, s_info)

std::string MyFilter::getName() const { return s_info.name; }

void MyFilter::addArgs(ProgramArgs& args)
{
    args.add("param", "Some parameter", m_value, 1.0);
}

void MyFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Intensity);
    m_myDimension = layout->registerOrAssignDim("MyDimension",
            Dimension::Type::Unsigned8);
}

PointViewSet MyFilter::run(PointViewPtr input)
{
    PointViewSet viewSet;
    viewSet.insert(input);
    return viewSet;
}

} // namespace pdal
