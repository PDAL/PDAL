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

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();

private:
    double m_value;
    Dimension::Id::Enum m_myDimension;

    virtual void addDimensions(PointLayoutPtr layout);
    virtual void processOptions(const Options& options);
    virtual PointViewSet run(PointViewPtr view);

    MyFilter& operator=(const MyFilter&); // not implemented
    MyFilter(const MyFilter&); // not implemented
};

} // namespace pdal
