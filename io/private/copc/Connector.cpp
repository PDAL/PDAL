/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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

#include "Connector.hpp"

#include <pdal/pdal_types.hpp>

namespace pdal
{
namespace copc
{

Connector::Connector(const std::string& filename, const StringMap& headers,
        const StringMap& query) :
    m_filename(filename), m_headers(headers), m_query(query), m_arbiter(new arbiter::Arbiter)
{}

std::vector<char> Connector::getBinary(uint64_t offset, int32_t size) const
{
    if (size <= 0)
        return std::vector<char>();

    if (m_arbiter->isLocal(m_filename))
    {
        std::vector<char> buf(size);
        std::ifstream in(m_filename, std::ios::binary);
        in.seekg(offset);
        in.read(buf.data(), size);
        return buf;
    }
    else
    {
        StringMap headers(m_headers);
        headers["Range"] = "bytes=" + std::to_string(offset) + "-" +
            std::to_string(offset + size - 1);
        return m_arbiter->getBinary(m_filename, headers, m_query);
    }
}

} // namespace copc
} // namespace pdal
