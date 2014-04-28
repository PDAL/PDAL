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

#ifndef INCLUDED_FILTERS_PREDICATEFILTER_HPP
#define INCLUDED_FILTERS_PREDICATEFILTER_HPP

#include <pdal/pdal_internal.hpp>
#ifdef PDAL_HAVE_PYTHON

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>

#include <pdal/plang/BufferedInvocation.hpp>


namespace pdal
{
namespace filters
{

class PDAL_DLL Predicate : public Filter
{
public:
    SET_STAGE_NAME("filters.predicate", "Predicate Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.predicate.html")  
#ifdef PDAL_HAVE_PYTHON
    SET_STAGE_ENABLED(true)
#else
    SET_STAGE_ENABLED(false)
#endif
    
    Predicate(Stage& prevStage, const Options&);
    ~Predicate();

    virtual void initialize();
    static Options getDefaultOptions();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        throw iterator_not_found(
            "filters.predicate random iterator not implemented");
    }

    boost::uint32_t processBuffer(PointBuffer& data, pdal::plang::BufferedInvocation&) const;

    const pdal::plang::Script& getScript() const
    {
        return *m_script;
    }

private:
    pdal::plang::Script* m_script;

    Predicate& operator=(const Predicate&); // not implemented
    Predicate(const Predicate&); // not implemented
};


namespace iterators
{
namespace sequential
{


class PDAL_DLL Predicate : public pdal::FilterSequentialIterator
{
public:
    Predicate(const pdal::filters::Predicate& filter, PointBuffer& buffer);
    ~Predicate();

    void read();

    boost::uint64_t getNumPointsProcessed() const
    {
        return m_numPointsProcessed;
    }
    boost::uint64_t getNumPointsPassed() const
    {
        return m_numPointsPassed;
    }

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    void readBeginImpl();
    boost::uint32_t readBufferImpl(PointBuffer&);
    void readEndImpl();
    bool atEndImpl() const;

    void createParser();

    const pdal::filters::Predicate& m_predicateFilter;
    pdal::plang::BufferedInvocation* m_pythonMethod;

    boost::uint64_t m_numPointsProcessed;
    boost::uint64_t m_numPointsPassed;
};

}
} // iterators::sequential



}
} // pdal::filteers

#endif

#endif
