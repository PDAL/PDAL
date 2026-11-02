#pragma once

#include <map>

#include <pdal/pdal_types.hpp>

#include <kernels/private/tindex/TIndexCommon.hpp>

namespace pdal
{
namespace tindex
{

// Re-reads a stac-geoparquet file with libarrow, and reconstructs
// string fields matching the `<structName>.<field>` pattern into
// nested structs.
void nestFieldsToStruct(const std::string& filename);

} // namespace tindex
} // namespace pdal