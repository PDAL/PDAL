/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#pragma once

#include <pdal/pdal_internal.hpp>

#ifdef PDAL_HAVE_NITRO
#include <pdal/drivers/las/Reader.hpp>

namespace pdal
{
namespace drivers
{
namespace nitf
{


class PDAL_DLL NitfReader : public las::Reader
{
public:
    SET_STAGE_NAME("drivers.nitf.reader", "NITF Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.nitf.reader.html")
    SET_STAGE_ENABLED(true)

    NitfReader() : las::Reader()
        {}

private:
    uint64_t m_offset;
    uint64_t m_length;

    virtual void initialize();
    virtual void ready(PointContextRef ctx);
    virtual StreamFactoryPtr createFactory() const
    {
        return StreamFactoryPtr(
            new FilenameSubsetStreamFactory(m_filename, m_offset, m_length));
    }

    NitfReader& operator=(const NitfReader&); // not implemented
    NitfReader(const NitfReader&); // not implemented
};

} // namespace nitf
} // namespace drivers
} // namespace pdal

#endif // PDAL_HAVE_NITRO
