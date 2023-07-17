// StraightenFilter.hpp

#pragma once
#include "libpdal_plugin_filter_straighten_export.h"
#include <pdal/Filter.hpp>
#include <pdal/Streamable.hpp>
#include <pdal/pdal_internal.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal {
class LIBPDAL_PLUGIN_FILTER_STRAIGHTEN_EXPORT StraightenFilter
    : public Filter,
      public Streamable {
public:
  StraightenFilter();

  StraightenFilter &operator=(const StraightenFilter &) = delete;
  StraightenFilter(const StraightenFilter &) = delete;

  std::string getName() const;

private:
  virtual void addArgs(ProgramArgs &args) override;
  virtual void initialize() override;
  virtual bool processOne(PointRef &point) override;
  virtual void filter(PointView &view) override;

  struct Args;
  std::unique_ptr<Args> m_args;
};

} // namespace pdal