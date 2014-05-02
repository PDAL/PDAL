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

#include <boost/numeric/conversion/cast.hpp>

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
public:
    Scaler() : scale(1.0), offset(0), size(0)
    {}

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
    SET_STAGE_ENABLED(true)


    Scaling(const Options& options) : Filter(options)
        {}
    Scaling& operator=(const Scaling&) = delete;
    Scaling(const Scaling&) = delete;

    static Options getDefaultOptions();
    virtual void initialize();

    pdal::StageSequentialIterator*
        createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator(PointBuffer&) const;
    
    std::vector<scaling::Scaler> const& getScalers() const
        { return m_scalers; }
    
    std::map<dimension::id, dimension::id> const& getScaleMap() const
        { return m_scale_map; }
    dimension::Interpretation getInterpretation(std::string t) const;
    
private:
    void checkImpedance();
    Schema alterSchema(Schema const& schema);

    std::vector<scaling::Scaler> m_scalers;
    std::map<dimension::id, dimension::id> m_scale_map;
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
    template<class T>
    void scale(Dimension const& from_dimension, Dimension const& to_dimension,
        T& value) const;

    std::map<boost::optional<pdal::Dimension const&>,
        boost::optional<pdal::Dimension const&> > m_dimension_map;
    void readBufferBeginImpl(PointBuffer&);
    boost::uint32_t readBufferImpl(PointBuffer&);    
    void scaleData(PointBuffer& buffer, boost::uint32_t numRead);

private:
    IteratorBase& operator=(IteratorBase const&);
};

} // scaling

namespace sequential
{


class PDAL_DLL Scaling : public pdal::FilterSequentialIterator,
    public scaling::IteratorBase
{
public:
    Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer);

protected:
    virtual void readBufferBeginImpl(PointBuffer& buffer)
        { scaling::IteratorBase::readBufferBeginImpl(buffer); }
    virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    bool atEndImpl() const;
};

} // namespace sequential

namespace random
{
    
class PDAL_DLL Scaling : public pdal::FilterRandomIterator,
    public scaling::IteratorBase
{
public:
    Scaling(const pdal::filters::Scaling& filter, PointBuffer& buffer);
    virtual ~Scaling() {};

protected:
    inline virtual void readBufferBeginImpl(PointBuffer& buffer)
        { scaling::IteratorBase::readBufferBeginImpl(buffer); }
    virtual boost::uint32_t readBufferImpl(PointBuffer& buffer);
    virtual boost::uint64_t seekImpl(boost::uint64_t);
};

} // namespace random
    
template <class T>
void scaling::IteratorBase::scale(Dimension const& from_dimension,
    Dimension const& to_dimension, T& value) const
{
    //ABELL - This bit was confusing to me.  What we appear to be doing is
    //  getting a value that assumes that we were going to scale with the old
    //  offset/scale and then instead apply the new scaling.
    //  In other words, scale up to "actual" value (from nominal) and then
    //  adjust to a new nominal value that assumes we'll get an actual value
    //  by applying the new scale factor.
    //ABELL -  Really don't understand why we don't just adjust the scale factor
    //  appropriately.
    double scaled = (value * from_dimension.getNumericScale() +
        from_dimension.getNumericOffset() - to_dimension.getNumericOffset()) /
        to_dimension.getNumericScale();
    // FIXME: This only downscales, not upscales. If
    // from_dimension.getNumericScale is > to_dimension.getNumericScale ==> BOOM

    std::string err;
    T errVal;
    try
    {
        value = boost::numeric_cast<T>(scaled);
        return;
    }
    catch (boost::numeric::positive_overflow)
    {
        err = "greater than std::numeric_limits::max()";
        errVal = std::numeric_limits<T>::max();
    }
    catch (boost::numeric::negative_overflow)
    {
        err = "less than std::numeric_limits::min()";
        errVal = std::numeric_limits<T>::min();
    }
    std::ostringstream oss;
    oss.precision(12);
    oss.setf(std::ios::fixed);
    oss << "scaling::IteratorBase::scale: '" <<
        to_dimension.getNumericScale() << "' and/or offset: " <<
        to_dimension.getNumericOffset() <<"' combination causes "
        "de-scaled value to be " << err << "for dimension '" <<
        to_dimension.getFQName() << "'. " << "(v - offset)/ scale) is: (" <<
        value << " - " << to_dimension.getNumericOffset()  << ")/" <<
        to_dimension.getNumericScale() <<") == '" << scaled <<
        "' but max() for the datatype is: " << errVal;
    throw std::out_of_range(oss.str());
} // namespace scaling

} // namespace iterators

} // namespace filters
} // namespace pdal

#endif
