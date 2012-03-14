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

//#include <Python.h>
//#include <numpy/arrayobject.h>


namespace pdal { namespace plang {


BufferedInvocation::BufferedInvocation(const std::string& source)
    : Invocation(source)
{
    return;
}



void BufferedInvocation::beginChunk(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    schema::Map const& map = schema.getDimensions();
    schema::index_by_index const& idx = map.get<schema::index>();
    for (schema::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;
        const std::string& name = dim.getName();

        boost::uint8_t* data = buffer.getData(0) + dim.getByteOffset();

        const boost::uint32_t numPoints = buffer.getNumPoints();
        const boost::uint32_t stride = buffer.getSchema().getByteSize();
        const dimension::Interpretation datatype = dim.getInterpretation();
        const boost::uint32_t numBytes = dim.getByteSize();
        this->insertArgument(name, data, numPoints, stride, datatype, numBytes);
    }

    return;
}


void BufferedInvocation::endChunk(PointBuffer& buffer)
{
    const Schema& schema = buffer.getSchema();

    schema::Map const& map = schema.getDimensions();
    schema::index_by_index const& idx = map.get<schema::index>();
    for (schema::index_by_index::const_iterator iter = idx.begin(); iter != idx.end(); ++iter)
    {
        const Dimension& dim = *iter;
        const std::string& name = dim.getName();
        
        if (hasOutputVariable(name))
        {
            boost::uint8_t* data = buffer.getData(0) + dim.getByteOffset();
            const boost::uint32_t numPoints = buffer.getNumPoints();
            const boost::uint32_t stride = buffer.getSchema().getByteSize();
            const dimension::Interpretation datatype = dim.getInterpretation();
            const boost::uint32_t numBytes = dim.getByteSize();
            extractResult(name, data, numPoints, stride, datatype, numBytes);
        }
    }

    return;
}



} } //namespaces

#endif
