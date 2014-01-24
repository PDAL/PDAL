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

#ifndef INCLUDED_FILTERS_SCALINGFILTER_HPP
#define INCLUDED_FILTERS_SCALINGFILTER_HPP

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

namespace scaling
{

struct PDAL_DLL Scaler
{

    std::string name;
    std::string type;
    double scale;
    double offset;
    boost::uint32_t size;
};

}

class PDAL_DLL Scaling: public Filter
{
public:
    SET_STAGE_NAME("filters.scaling", "Scaling Filter")
    SET_STAGE_LINK("http://pdal.io/stages/filters.scaling.html")  

    Scaling(Stage& prevStage, const Options&);

    static Options getDefaultOptions();
    virtual void initialize();

    bool supportsIterator(StageIteratorType t) const
    {
        if (t == StageIterator_Sequential) return true;
        if (t == StageIterator_Random) return true;

        return false;
    }

    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const;
    
    std::vector<scaling::Scaler> const& getScalers() const
    {
        return m_scalers;
    }
    
    std::map<dimension::id, dimension::id> const& getScaleMap() const { return m_scale_map; }
    dimension::Interpretation getInterpretation(std::string const& t) const;
    
private:
    void checkImpedance();
    
    Schema alterSchema(Schema const& schema);

    Scaling& operator=(const Scaling&); // not implemented
    Scaling(const Scaling&); // not implemented

    std::vector<scaling::Scaler> m_scalers;

    std::map<dimension::id, dimension::id> m_scale_map;
;
};


namespace iterators
{
    

namespace scaling
{
    
class PDAL_DLL IteratorBase
{

public:
    IteratorBase(const pdal::filters::Scaling& filter, PointBuffer& buffer);    

protected:
    const pdal::filters::Scaling& m_scalingFilter;
    void writeScaledData(PointBuffer& buffer,
                         Dimension const& from_dimension,
                         Dimension const& to_dimension,
                         boost::uint32_t pointIndex);
    template<class T> void scale(Dimension const& from_dimension,
                                 Dimension const& to_dimension,
                                 T& value) const;

    std::map<boost::optional<pdal::Dimension const&>, boost::optional<pdal::Dimension const&> > m_dimension_map;

    void readBufferBeginImpl(PointBuffer&);
    boost::uint32_t readBufferImpl(PointBuffer&);    

    void scaleData(PointBuffer& buffer, boost::uint32_t numRead);

private:
    IteratorBase& operator=(IteratorBase const&);
};

} // scaling

namespace sequential
{


class PDAL_DLL Scaling : public pdal::FilterSequentialIterator, public scaling::IteratorBase
{
public:
    Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer);

protected:

    inline virtual void readBufferBeginImpl(PointBuffer& buffer) { scaling::IteratorBase::readBufferBeginImpl(buffer); }
    virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);
    

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    bool atEndImpl() const;

};

} // namespace sequential

namespace random
{
    
class PDAL_DLL Scaling : public pdal::FilterRandomIterator, public scaling::IteratorBase
{
public:
    Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer);
    virtual ~Scaling() {};

protected:
    inline virtual void readBufferBeginImpl(PointBuffer& buffer) { scaling::IteratorBase::readBufferBeginImpl(buffer); }
    virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);
    
    virtual boost::uint64_t seekImpl(boost::uint64_t);

};

} // namespace random
    
#ifdef PDAL_COMPILER_MSVC
// when template T is know, std::numeric_limits<T>::is_exact is a constant...
#  pragma warning(push)
#  pragma warning(disable: 4127)  // conditional expression is constant
#endif

template <class T>
inline void scaling::IteratorBase::scale(Dimension const& from_dimension,
                           Dimension const& to_dimension,
                           T& value) const
{

    double v = static_cast<double>(value);
    double scaled = (v*from_dimension.getNumericScale() + from_dimension.getNumericOffset() - to_dimension.getNumericOffset())/to_dimension.getNumericScale();
    // FIXME: This only downscales, not upscales. If from_dimension.getNumericScale is > to_dimension.getNumericScale ==> BOOM

    T output(0);
    boost::int64_t i64(0);
    boost::uint64_t u64(0);
    if (boost::is_floating_point<T>::value)
    {   
        // Since we're a float already, just 
        // cast it and be done with it.
        value = static_cast<T>(scaled);
        return;
    }

    bool bSigned(boost::is_signed<T>::value);
    bool bGreater(false);
    bool bLess(false);
    if (bSigned)
    {
        i64 = static_cast<boost::int64_t>(scaled);
        boost::int64_t mn = static_cast<boost::int64_t>((std::numeric_limits<T>::min)());            
        boost::int64_t mx = static_cast<boost::int64_t>((std::numeric_limits<T>::max)());            
        bGreater = (i64 > mx);
        bLess = (i64 < mn);
        output = static_cast<T>(i64);              
        if (!bGreater && !bLess)
        {
            // We didn't overflow, we're done
            value = output;
            return;
        }
    }
    else
    {
        u64 = static_cast<boost::uint64_t>(scaled);
        boost::uint64_t mn = static_cast<boost::uint64_t>((std::numeric_limits<T>::min)());
        boost::uint64_t mx = static_cast<boost::uint64_t>((std::numeric_limits<T>::max)());
        bGreater = (u64 > mx);
        bLess = (u64 < mn);
        output = static_cast<T>(u64);
        if (!bGreater && !bLess)
        {
            // We didn't overflow, we're done
            value = output;
            return;
        }
    }


    if (bGreater)
    {
        std::ostringstream oss;
        boost::int64_t out(0);
        if (bSigned)
            out = static_cast<boost::int64_t>(i64);
        else
            out = static_cast<boost::int64_t>(u64);
        
        oss.precision(12);
        oss.setf(std::ios::fixed);
        oss << "scaling::IteratorBase::scale: '" << to_dimension.getNumericScale() 
            << "' and/or offset: " << to_dimension.getNumericOffset() <<"' combination causes "
            "de-scaled value to be greater than std::numeric_limits::max for dimension '" 
            << to_dimension.getFQName() << "'. " <<
            "(v - offset)/ scale) is: (" 
            << v << " - " << to_dimension.getNumericOffset()  << ")/" 
            << to_dimension.getNumericScale() <<") == '" << out 
            << "' but max() for the datatype is: " 
            << (std::numeric_limits<T>::max)();
        throw std::out_of_range(oss.str());
    }
    else if (bLess)
    {
        std::ostringstream oss;
        boost::int64_t out(0);
        if (bSigned)
            out = static_cast<boost::int64_t>(i64);
        else
            out = static_cast<boost::int64_t>(u64);
        oss.precision(12);
        oss.setf(std::ios::fixed);
        oss << "scaling::IteratorBase::scale: '" << to_dimension.getNumericScale() 
            << "' and/or offset: " << to_dimension.getNumericOffset() <<"' combination causes "
            "de-scaled value to be less than std::numeric_limits::mine for dimension '" 
            << to_dimension.getFQName() << "'. " <<
            "(v - offset)/ scale) is: (" 
            << v << " - " << to_dimension.getNumericOffset()  << ")/" 
            << to_dimension.getNumericScale() <<") == '" << out 
            << "' but min() for the datatype is: " 
            << (std::numeric_limits<T>::min)();

        throw std::out_of_range(oss.str());
    }


    return;
}

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

}


}
} // namespaces

#endif
