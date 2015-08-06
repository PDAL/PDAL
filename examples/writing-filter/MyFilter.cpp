// MyFilter.cpp

#include "MyFilter.hpp"

#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/StageFactory.hpp>

CREATE_FILTER_PLUGIN(myfilter, MyFilter)

void MyFilter::processOptions(const pdal::Options& options)
{
  m_value = options.getValueOrDefault<double>("param", 1.0);
}

void MyFilter::addDimensions(PointContextRef ctx)
{
  ctx.registerDim(pdal::Dimension::Id::Intensity);
}

PointBufferSet MyFilter::run(PointBufferPtr input)
{
  PointBufferSet pbSet;
  pbSet.insert(input);
  return pbSet;
}

