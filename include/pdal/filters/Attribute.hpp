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

#ifndef INCLUDED_FILTERS_ATTRIBUTEFILTER_HPP
#define INCLUDED_FILTERS_ATTRIBUTEFILTER_HPP

#include <pdal/Filter.hpp>
#include <pdal/FilterIterator.hpp>


namespace pdal
{
    class PointBuffer;
}

// #include <boost/config/warning_disable.hpp>
// #include <boost/spirit/include/qi.hpp>
// 
// namespace parser
// {
//     namespace qi = boost::spirit::qi;
//     namespace ascii = boost::spirit::ascii;
// 
//     ///////////////////////////////////////////////////////////////////////////
//     //  Our number list parser
//     ///////////////////////////////////////////////////////////////////////////
//     //[tutorial_numlist1
//     template <typename Iterator>
//     bool parse_doubles(Iterator first, Iterator last)
//     {
//         using qi::double_;
//         using qi::phrase_parse;
//         using ascii::space;
// 
//         bool r = phrase_parse(
//             first,                          /*< start iterator >*/
//             last,                           /*< end iterator >*/
//             double_ >> *(',' >> double_),   /*< the parser >*/
//             space                           /*< the skip-parser >*/
//         );
//         if (first != last) // fail if we did not get a full match
//             return false;
//         return r;
//     }
//     //]
// }


namespace pdal { namespace filters {

class AttributeFilterSequentialIterator;

class PDAL_DLL Attribute : public Filter
{
public:
    SET_STAGE_NAME("filters.attribute", "Attribute Filter")

    Attribute(Stage& prevStage, const Options&);

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator() const;
    pdal::StageRandomIterator* createRandomIterator() const { return NULL; }

    void processBuffer(PointBuffer& data) const;

private:

    Attribute& operator=(const Attribute&); // not implemented
    Attribute(const Attribute&); // not implemented
};


namespace iterators { namespace sequential {
    

class Attribute : public pdal::FilterSequentialIterator
{
public:
    Attribute(const pdal::filters::Attribute& filter);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::filters::Attribute& m_attributeFilter;
};

} } // iterators::sequential



} } // pdal::filteers

#endif
