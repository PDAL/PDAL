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

#include <pdal/filters/Predicate.hpp>

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{


Predicate::~Predicate()
{
    delete m_script;
}


void Predicate::initialize()
{
    Filter::initialize();

    m_script = new pdal::plang::Script(getOptions());

    log()->get(logDEBUG)  << "script " << *m_script << std::endl;

    return;
}


Options Predicate::getDefaultOptions()
{
    Options options;

    Option script("script", "");
    options.add(script);

    Option module("module", "");
    options.add(module);

    Option function("function", "");
    options.add(function);

    return options;
}


boost::uint32_t Predicate::processBuffer(PointBuffer& data, pdal::plang::BufferedInvocation& python) const
{
    python.resetArguments();

    python.beginChunk(data);

    python.execute();

    if (!python.hasOutputVariable("Mask"))
    {
        throw python_error("Mask variable not set in predicate filter function");
    }

    boost::uint8_t* mask = new boost::uint8_t[data.getNumPoints()];

    PointBuffer dstData(data.getSchema(), data.getCapacity());

    python.extractResult("Mask", (boost::uint8_t*)mask, data.getNumPoints(), 1, pdal::dimension::RawByte, 1);

    boost::uint8_t* dst = dstData.getData(0);
    boost::uint8_t* src = data.getData(0);

    const Schema& schema = dstData.getSchema();
    boost::uint32_t numBytes = schema.getByteSize();
    assert(numBytes == data.getSchema().getByteSize());

    boost::uint32_t numSrcPoints = data.getNumPoints();
    boost::uint32_t count = 0;
    for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    {
        if (mask[srcIndex])
        {
            memcpy(dst, src, numBytes);
            dst += numBytes;
            ++count;
            dstData.setNumPoints(count);
        }
        src += numBytes;
    }


    data.copyPointsFast(0, 0, dstData, count);
    data.setNumPoints(count);

    delete[] mask;

    return count;
}


pdal::StageSequentialIterator* Predicate::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Predicate(*this, buffer);
}


//---------------------------------------------------------------------------


namespace iterators
{
namespace sequential
{

Predicate::Predicate(const pdal::filters::Predicate& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_predicateFilter(filter)
    , m_pythonMethod(NULL)
    , m_numPointsProcessed(0)
    , m_numPointsPassed(0)
{
    return;
}


Predicate::~Predicate()
{
    delete m_pythonMethod;
}


void Predicate::createParser()
{
    const pdal::plang::Script& script = m_predicateFilter.getScript();

    m_pythonMethod = new pdal::plang::BufferedInvocation(script);

    m_pythonMethod->compile();

    return;
}


void Predicate::readBeginImpl()
{
    if (!m_pythonMethod)
    {
        createParser();
    }

    m_numPointsProcessed = m_numPointsPassed = 0;

    pdal::GlobalEnvironment::get().getPythonEnvironment().set_stdout(m_predicateFilter.log()->getLogStream());

    return;
}


void Predicate::readEndImpl()
{
    pdal::GlobalEnvironment::get().getPythonEnvironment().reset_stdout();

    return;
}


boost::uint32_t Predicate::readBufferImpl(PointBuffer& dstData)
{
    // read in a full block of points

    const boost::uint32_t numRead = getPrevIterator().read(dstData);
    if (numRead > 0)
    {
        m_numPointsProcessed = dstData.getNumPoints();

        // copies the points as they pass in-place
        m_predicateFilter.processBuffer(dstData, *m_pythonMethod);
    }

    m_numPointsPassed = dstData.getNumPoints();

    return dstData.getNumPoints();
}


boost::uint64_t Predicate::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Predicate::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // iterators::sequential

}
} // pdal::filters

#endif
