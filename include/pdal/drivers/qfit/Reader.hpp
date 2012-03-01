/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#ifndef INCLUDED_PDAL_DRIVER_QFIT_READER_HPP
#define INCLUDED_PDAL_DRIVER_QFIT_READER_HPP

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/Options.hpp>

#include <pdal/StageIterator.hpp>


#include <vector>

#include <boost/detail/endian.hpp>

#ifdef BOOST_LITTLE_ENDIAN
# define QFIT_SWAP_BE_TO_LE(p) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + sizeof(p) - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)

# define QFIT_SWAP_BE_TO_LE_N(p, n) \
    do { \
        char* first = static_cast<char*>(static_cast<void*>(&p)); \
        char* last = first + n - 1; \
        for(; first < last; ++first, --last) { \
            char const x = *last; \
            *last = *first; \
            *first = x; \
        }} while(false)
#endif

namespace pdal { namespace drivers { namespace qfit {


enum QFIT_Format_Type
{
    QFIT_Format_10 = 10,
    QFIT_Format_12 = 12,
    QFIT_Format_14 = 14,
    QFIT_Format_Unknown = 128
};

class qfit_error : public pdal_error
{
public:

    qfit_error(std::string const& msg)
        : pdal_error(msg)
    {}
};

class PointIndexes
{
public:
    PointIndexes(const Schema& schema, QFIT_Format_Type format);
    int Time;
    int X;
    int Y;
    int Z;
    
    int StartPulse;
    int ReflectedPulse;
    int ScanAngleRank;
    int Pitch;
    int Roll;
    int PDOP;
    int PulseWidth;
    int GPSTime;
    int PassiveSignal;
    int PassiveX;
    int PassiveY;
    int PassiveZ;
    pdal::Dimension dimX;
    
};


class PointDimensions
{
public:
    PointDimensions(const Schema& schema);
    
    pdal::Dimension const* Time;
    pdal::Dimension const* X;
    pdal::Dimension const* Y;
    pdal::Dimension const* Z;
    
    pdal::Dimension const* StartPulse;
    pdal::Dimension const* ReflectedPulse;
    pdal::Dimension const* ScanAngleRank;
    pdal::Dimension const* Pitch;
    pdal::Dimension const* Roll;
    pdal::Dimension const* PDOP;
    pdal::Dimension const* PulseWidth;
    pdal::Dimension const* GPSTime;
    
    pdal::Dimension const* PassiveSignal;
    
    pdal::Dimension const* PassiveX;
    pdal::Dimension const* PassiveY;
    pdal::Dimension const* PassiveZ;
};

//
// supported options:
//   <uint32>id
//   <bool>debug
//   <uint32>verbose
//   <string>filename  [required]
//

class PDAL_DLL Reader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.qfit.reader", "QFIT Reader")

    Reader(const Options& options);
    ~Reader();
    
    virtual void initialize();
    virtual const Options getDefaultOptions() const;
    virtual void addDefaultDimensions();
    
    std::string getFileName() const;

    bool supportsIterator (StageIteratorType t) const
    {   
        if (t == StageIterator_Sequential ) return true;
        if (t == StageIterator_Random ) return true;
        
        return false;
    }
    
    pdal::StageSequentialIterator* createSequentialIterator(PointBuffer& buffer) const;
    pdal::StageRandomIterator* createRandomIterator() const;
    
    std::size_t getPointDataOffset() const { return m_offset; }
    boost::uint32_t getPointDataSize() const { return m_size; }

    // this is called by the stage's iterator
    boost::uint32_t processBuffer(PointBuffer& PointBuffer, std::istream& stream, boost::uint64_t numPointsLeft) const;

    // for dumping
    virtual boost::property_tree::ptree toPTree() const;

protected:
    inline QFIT_Format_Type getFormat() const { return m_format; }

private:

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented

    QFIT_Format_Type m_format;
    std::size_t m_offset;
    boost::uint32_t m_size;
    bool m_flip_x;
    double m_scale_z;
    bool m_littleEndian;
    
    void registerFields();


};

namespace iterators {

namespace sequential {

class Reader : public pdal::ReaderSequentialIterator
{
public:
    Reader(const pdal::drivers::qfit::Reader& reader, PointBuffer& buffer);
    ~Reader();

private:
    boost::uint64_t skipImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);
    bool atEndImpl() const;

    const pdal::drivers::qfit::Reader& m_reader;
    std::istream* m_istream;
};

    
} // sequential

namespace random {

class Reader : public pdal::ReaderRandomIterator
{
public:
    Reader(const pdal::drivers::qfit::Reader& reader);
    ~Reader();

private:
    boost::uint64_t seekImpl(boost::uint64_t);
    boost::uint32_t readBufferImpl(PointBuffer&);

    const pdal::drivers::qfit::Reader& m_reader;
    std::istream* m_istream;
};

} // random
} // iterators
}}} // namespace pdal::driver::qfit


#endif // INCLUDED_PDAL_DRIVER_QFIT_READER_HPP
