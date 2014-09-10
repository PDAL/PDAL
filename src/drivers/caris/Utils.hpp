/************************************************************************
 * Copyright (c) 2012, CARIS
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of CARIS nor the names of its contributors may be
 *     used to endorse or promote products derived from this software without
 *     specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ************************************************************************/
#ifndef INCLUDED_DRIVERS_CSAR_UTILS_HPP
#define INCLUDED_DRIVERS_CSAR_UTILS_HPP

#include "config.h"
#include "caris/caris_pc_wrapper.h"

#ifdef _MSC_VER
#   pragma warning(push, 3)
#   pragma warning(disable : DISABLED_3RDPARTY_WARNINGS)
#endif

#include <map>
#include <boost/optional.hpp>

#include <pdal/Dimension.hpp>

#ifdef _MSC_VER
#   pragma warning(pop)
#endif

namespace pdal {
namespace csar {
namespace utils {

pdal::Dimension::Type::Enum carisTypeToPdal(caris_type in_type);
//pdal::dimension::size_type carisTypeToSize(caris_type in_type);
std::string systemPathToURI(std::string const& in_path);

/// Lookup an item in a std::map
///
/// @tparam T,U map template paramaters, implied
/// @param in_map map to check
/// @param in_key key to lookup
/// @return An optional reference to the value of \e in_key
///         none if \e in_key doesn't exist in \e in_map
template<typename T, typename U>
boost::optional<U const&> getOptionalCRef(
    std::map<T, U> const& in_map, 
    T const& in_key)
{
    typename std::map<T, U>::const_iterator itr = in_map.find(in_key);
    if(itr != in_map.end())
        return itr->second;
    else
        return boost::none;
}

} // namespace utils
} // namespace csar
} // namespace pdal

#endif
