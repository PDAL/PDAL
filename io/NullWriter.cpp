
#include "NullWriter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.null",
    "Null writer.  Provides a sink for points in a pipeline.  "\
        "It's the same as sending pipeline output to /dev/null.",
    "https://pdal.org/stages/writers.null.html"
};

CREATE_STATIC_STAGE(NullWriter, s_info)

std::string NullWriter::getName() const { return s_info.name; }

} // namespace pdal
