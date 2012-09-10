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

#ifndef INCLUDED_FILTERS_SELECTORFILTER_HPP
#define INCLUDED_FILTERS_SELECTORFILTER_HPP

#include <boost/shared_ptr.hpp>

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>
#include <pdal/Utils.hpp>

#include <map>

namespace pdal
{
class PointBuffer;
}

namespace pdal
{
namespace filters
{

class PDAL_DLL Selector: public Filter
{
public:
    SET_STAGE_NAME("filters.selector", "Dimension Selection Filter")

    Selector(Stage& prevStage, const Options&);

    virtual const Options getDefaultOptions() const;
    virtual void initialize();

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const
    {
        return NULL;
    }
    
    inline std::map<std::string, bool> const& getIgnoredMap() const { return m_ignoredMap; }
    
    inline bool doIgnoreUnspecifiedDimensions() const { return m_ignoreDefault; }
    std::vector<Dimension> const& getCreatedDimensions() const { return m_createDimensions; }
    
private:
    void checkImpedance();


    Selector& operator=(const Selector&); // not implemented
    Selector(const Selector&); // not implemented
    
    std::map<std::string, bool> m_ignoredMap;
    bool m_ignoreDefault;
    std::vector<Dimension> m_createDimensions;
};


namespace iterators
{
namespace sequential
{


class PDAL_DLL Selector : public pdal::FilterSequentialIterator
{
public:
    Selector(const pdal::filters::Selector& filter, PointBuffer& buffer);


private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;
    void alterSchema(pdal::PointBuffer&);
    const pdal::filters::Selector& m_selectorFilter;

};


}
} // namespaces

}
} // namespaces

#endif
