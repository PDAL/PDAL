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

#include <pdal/filters/Programmable.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{


Programmable::Programmable(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_script(NULL)
{
    return;
}


Programmable::~Programmable()
{
    delete m_script;
}


void Programmable::initialize()
{
    Filter::initialize();

    m_script = new pdal::plang::Script(getOptions());

    log()->get(logDEBUG)  << "script " << *m_script << std::endl;

    return;
}


const Options Programmable::getDefaultOptions() const
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


void Programmable::processBuffer(PointBuffer& data, pdal::plang::BufferedInvocation& python) const
{
    python.resetArguments();

    python.beginChunk(data);

    python.execute();

    python.endChunk(data);

    return;
}


pdal::StageSequentialIterator* Programmable::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Programmable(*this, buffer);
}


//---------------------------------------------------------------------------


namespace iterators
{
namespace sequential
{


Programmable::Programmable(const pdal::filters::Programmable& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_programmableFilter(filter)
    , m_pythonMethod(NULL)
{
    return;
}


Programmable::~Programmable()
{
    delete m_pythonMethod;
}


void Programmable::createParser()
{
    const pdal::plang::Script& script = m_programmableFilter.getScript();

    m_pythonMethod = new pdal::plang::BufferedInvocation(script);

    m_pythonMethod->compile();

    return;
}


boost::uint32_t Programmable::readBufferImpl(PointBuffer& data)
{
    if (!m_pythonMethod)
    {
        createParser();
    }

    const boost::uint32_t numRead = getPrevIterator().read(data);

    m_programmableFilter.processBuffer(data, *m_pythonMethod);

    return numRead;
}


boost::uint64_t Programmable::skipImpl(boost::uint64_t count)
{
    getPrevIterator().skip(count);
    return count;
}


bool Programmable::atEndImpl() const
{
    return getPrevIterator().atEnd();
}

}
} // iterators::sequential

}
} // pdal::filters

#endif
