/******************************************************************************
 * Copyright (c) 2020, Hobu Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following
 * conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 ****************************************************************************/

#pragma warning (push)
#pragma warning (disable: 4251)
#include <lazperf/readers.hpp>
#pragma warning (pop)

#include <io/LasReader.hpp>
#include <io/private/las/Header.hpp>

#include "../connector/Connector.hpp"
#include "Tile.hpp"

namespace pdal
{
namespace copc
{

void Tile::read()
{
    try
    {
        std::vector<char> buf = m_connector.getBinary(m_entry.m_offset, m_entry.m_byteSize);
        lazperf::reader::chunk_decompressor d(m_header.pointFormat(), m_header.ebCount(),
            buf.data());

        // Resize our vector to accommodate the decompressed data.
        m_data.resize(m_entry.m_pointCount * m_header.pointSize);

        int32_t cnt = m_entry.m_pointCount;
        char *p = m_data.data();
        while (cnt--)
        {
            d.decompress(p);
            p += m_header.pointSize;
        }
    }
    catch (const std::exception& ex)
    {
        m_error = ex.what();
    }
    catch (...)
    {
        m_error = "Unknown exception when reading tile contents";
    }
}

} // namespace copc
} // namespace pdal

