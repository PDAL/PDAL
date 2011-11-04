/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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


#include <pdal/drivers/oci/common.hpp>

#include <iostream>

#include <pdal/Bounds.hpp>
#include <pdal/Utils.hpp>

namespace pdal { namespace drivers { namespace oci {

Block::Block(Connection connection)
    : num_points(0)
    , chunk(new std::vector<boost::uint8_t>)
    , m_connection(connection)
{
    m_connection->CreateType(&blk_extent);
    m_connection->CreateType(&blk_extent->sdo_ordinates, m_connection->GetOrdinateType());
    m_connection->CreateType(&blk_extent->sdo_elem_info, m_connection->GetElemInfoType());
    m_connection->CreateType(&blk_domain);
    m_connection->CreateType(&pc);
}

Block::~Block()
{
    m_connection->DestroyType(&blk_domain);
    m_connection->DestroyType(&blk_extent->sdo_elem_info);
    m_connection->DestroyType(&blk_extent->sdo_ordinates);
    m_connection->DestroyType(&pc);
    // FIXME: For some reason having the dtor destroy this
    // causes a segfault
    // m_connection->DestroyType(&blk_extent);
}



}}} // namespace pdal::driver::oci



