#pragma once

#include <map>

#ifdef PDAL_HAVE_ARROW
#include <arrow/type_fwd.h>
#include <arrow/io/type_fwd.h>
#include <arrow/ipc/type_fwd.h>
#endif // PDAL_HAVE_ARROW

#include <pdal/pdal_types.hpp>

#include "TIndexError.hpp"

namespace pdal
{
namespace tindex
{

void nestFieldsToStruct(const std::string& filename);

} // namespace tindex
} // namespace pdal