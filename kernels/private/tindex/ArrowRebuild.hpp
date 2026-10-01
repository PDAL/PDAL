
#include <map>

#include <arrow/type_fwd.h>
#include <arrow/io/type_fwd.h>
#include <arrow/ipc/type_fwd.h>

#include <pdal/pdal_types.hpp>

#include "TIndexError.hpp"

namespace pdal
{
namespace tindex
{

void nestFieldsToStruct(const std::string& filename);

} // namespace tindex
} // namespace pdal