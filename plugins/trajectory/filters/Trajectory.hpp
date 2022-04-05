#pragma once

#include <memory>

#include <pdal/Filter.hpp>

namespace pdal
{

class PDAL_DLL Trajectory : public Filter
{
    struct PrivateArgs;

public:
    Trajectory();
    ~Trajectory();

    std::string getName() const;

private:
    std::unique_ptr<PrivateArgs> m_args;

    virtual void addArgs(ProgramArgs& args);
    virtual void addDimensions(PointLayoutPtr layout);
    virtual PointViewSet run(PointViewPtr view);
};

} // namespace pdal

