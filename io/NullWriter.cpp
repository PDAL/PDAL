
#include "NullWriter.hpp"

namespace pdal
{

static StaticPluginInfo const NullWriter_info
{
    "writers.null",
    "Null writer.  Provides a sink for points in a pipeline.  "\
        "It's the same as sending pipeline output to /dev/null.",
    "https://pdal.org/stages/writers.null.html"
};

CREATE_STATIC_STAGE(NullWriter, NullWriter_info)

std::string NullWriter::getName() const { return NullWriter_info.name; }

} // namespace pdal
