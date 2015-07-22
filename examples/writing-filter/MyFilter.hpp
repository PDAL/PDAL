// MyFilter.hpp

#pragma once

#include <pdal/Filter.hpp>
#include <pdal/Stage.hpp>

#include <memory>
#include <set>

namespace pdal
{
class Options;
class PointBuffer;
class PointContext;
}

typedef std::shared_ptr<pdal::PointBuffer> PointBufferPtr;
typedef std::set<PointBufferPtr> PointBufferSet;
typedef PointContext PointContextRef;

class MyFilter : public pdal::Filter
{
public:
  SET_STAGE_NAME ("filters.name", "My Awesome Filter")
  SET_STAGE_LINK ("http://link/to/documentation")
  SET_PLUGIN_VERSION("1.0.0")

  MyFilter() : Filter() {};

private:
  virtual void addDimensions(PointContextRef ctx);
  virtual void processOptions(const Options& options);
  virtual PointBufferSet run(PointBufferPtr buf);

  MyFilter& operator=(const MyFilter&); // not implemented
  MyFilter(const MyFilter&); // not implemented
};

