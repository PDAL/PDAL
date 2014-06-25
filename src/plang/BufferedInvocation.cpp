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

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/plang/BufferedInvocation.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4127)  // conditional expression is constant
#  pragma warning(disable: 4505)  // unreferenced local function has been removed
#endif

namespace pdal
{
namespace plang
{


BufferedInvocation::BufferedInvocation(const Script& script)
    : Invocation(script)
{
    return;
}


void BufferedInvocation::beginChunk(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    schema::Map const& map = schema.getDimensions();
    schema::index_by_index const& idx = map.get<schema::index>();
    for (auto iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;

        //ABELL - does the interface allow us to use a fixed-size buffer
        //  and then call beginChunk in a loop or something similar?
        void *data = malloc(dim.getByteSize() * buffer.size());
        m_buffers.push_back(data);  // Hold pointer for deallocation
        char *p = (char *)data;
        for (PointId idx = 0; idx < buffer.size(); ++idx)
        {
            buffer.getRawField(dim, idx, (void *)p);
            p += dim.getByteSize();
        }

        const std::string& name = dim.getName();
        const boost::uint32_t numPoints = buffer.getNumPoints();
        const boost::uint32_t stride = dim.getByteSize();
        const dimension::Interpretation datatype = dim.getInterpretation();
        const boost::uint32_t numBytes = dim.getByteSize();
        insertArgument(name, (uint8_t *)data, numPoints, stride,
            datatype, numBytes);
    }
}


void BufferedInvocation::endChunk(PointBuffer& buffer)
{
    // for each entry in the script's outs dictionary,
    // look up that entry's name in the schema and then
    // copy the data into the right dimension spot in the
    // buffer

    std::vector<std::string> names;
    getOutputNames(names);

    const Schema& schema = buffer.getSchema();

    for (size_t i = 0; i < names.size(); i++)
    {
        schema::Map map = schema.getDimensions();
        schema::index_by_name& name_index = map.get<schema::name>();
        schema::index_by_name::const_iterator it = name_index.find(names[i]);
        if (it != name_index.end())
        {
            const Dimension& dim = *it;

            const std::string& name = dim.getName();
            assert(name == names[i]);
            assert(hasOutputVariable(name));
            const dimension::Interpretation datatype = dim.getInterpretation();
            const boost::uint32_t numBytes = dim.getByteSize();
            void *data = extractResult(name, datatype, numBytes);
            char *p = (char *)data;
            for (PointId idx = 0; idx < buffer.size(); ++idx)
            {
                buffer.setRawField(dim, idx, (void *)p);
                p += dim.getByteSize();
            }
        }
    }
    for (auto bi = m_buffers.begin(); bi != m_buffers.end(); ++bi)
        free(*bi);
    m_buffers.clear();
}

}
} //namespaces

#endif
