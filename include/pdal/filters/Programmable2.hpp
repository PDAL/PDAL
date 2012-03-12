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

#ifndef INCLUDED_FILTERS_PROGRAMMABLE2FILTER_HPP
#define INCLUDED_FILTERS_PROGRAMMABLE2FILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/plang/PythonSupport.hpp>


namespace pdal { namespace filters {

class Programmable2FilterSequentialIterator;

class PDAL_DLL Programmable2 : public Filter
{
public:
    SET_STAGE_NAME("filters.programmable2", "Programmable2 Filter")

    Programmable2(Stage& prevStage, const Options&);

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const { return NULL; }

    void processBuffer(PointBuffer& data, pdal::plang::PythonMethod& python) const;

    const std::string& getProgram() const { return m_program; }

private:
    std::string m_program;

    Programmable2& operator=(const Programmable2&); // not implemented
    Programmable2(const Programmable2&); // not implemented
};


namespace iterators { namespace sequential {
    

class PDAL_DLL Programmable2 : public pdal::FilterSequentialIterator
{
public:
    Programmable2(const pdal::filters::Programmable2& filter, PointBuffer& buffer);
    ~Programmable2();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    void createParser();
    
    const pdal::filters::Programmable2& m_programmableFilter;

    pdal::plang::PythonEnvironment* m_pythonEnv;
    pdal::plang::PythonMethod* m_pythonMethod;
};

} } // iterators::sequential



} } // pdal::filteers

#endif
