//
// (c) 2022 SRI International
//
#pragma once

#include <memory>

#include <pdal/Filter.hpp>

namespace pdal
{

namespace trajectory
{
    struct Args;
}

class PDAL_EXPORT Trajectory : public Filter
{
public:
    Trajectory();
    ~Trajectory();

    std::string getName() const;

private:
    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual PointViewSet run(PointViewPtr view);

    PointViewSet runAlgorithm(PointViewPtr view);

    std::unique_ptr<trajectory::Args> m_args;
};

} // namespace pdal

