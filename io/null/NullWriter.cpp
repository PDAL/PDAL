#include "NullWriter.hpp"

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.null",
    "Null writer.  Provides a sink for points in a pipeline.  "\
        "It's the same as sending pipeline output to /dev/null.",
    "http://pdal.io/stages/writers.null.html" );

CREATE_STATIC_PLUGIN(1, 0, NullWriter, Writer, s_info)

std::string NullWriter::getName() const { return s_info.name; }

}
