// MyFilter.cpp

#include "MyFilter.hpp"

#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("filters.name", "My awesome filter",
               "http://link/to/documentation");

CREATE_STATIC_PLUGIN(1, 0, MyFilter, Filter, s_info)

std::string MyFilter::getName() const
{
    return s_info.name;
}

Options MyFilter::getDefaultOptions()
{
    Options options;
    options.add("param", 1.0, "My parameter");
    return options;
}

void MyFilter::processOptions(const pdal::Options& options)
{
    m_value = options.getValueOrDefault<double>("param", 1.0);
}

void MyFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Intensity);
    m_myDimension = layout->registerOrAssignDim("MyDimension", Dimension::Type::Unsigned8);
}

PointViewSet MyFilter::run(PointViewPtr input)
{
    PointViewSet viewSet;
    viewSet.insert(input);
    return viewSet;
}

} // namespace pdal
