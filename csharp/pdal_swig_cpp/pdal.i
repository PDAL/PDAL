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
 
%module pdal_swig_cpp

%{
#include <iostream>

#include "pdal/pdal_config.hpp"

#include "pdal/Bounds.hpp"
#include "pdal/Range.hpp"

#include "pdal/Dimension.hpp"
#include "pdal/DimensionLayout.hpp"
#include "pdal/Schema.hpp"
#include "pdal/SchemaLayout.hpp"
#include "pdal/PointBuffer.hpp"
#include "pdal/SpatialReference.hpp"

#include "pdal/StageIterator.hpp"
#include "pdal/Reader.hpp"
#include "pdal/Writer.hpp"
#include "pdal/Filter.hpp"
#include "pdal/MultiFilter.hpp"
#include "pdal/Options.hpp"

//#include "pdal/DecimationFilter.hpp"

#include "pdal/drivers/las/Reader.hpp"
#include "pdal/drivers/las/Writer.hpp"
%}

%include "typemaps.i"

// C# support for std::string
%include "std_string.i"

// C# support for std::vector<T>
%include "std_vector.i"
namespace std
{
   %template(std_vector_u8) vector<unsigned char>;
   %template(std_vector_double) vector<double>;
   %template(std_vector_Dimension) vector<pdal::Dimension>;
   %template(std_vector_Range_double) vector<pdal::Range<double> >;
};
 

// fix up some missing types
namespace std
{
    typedef unsigned int size_t;
};

%include "std/std_iostream.i"

namespace boost
{
    typedef unsigned char uint8_t;
    typedef signed char int8_t;
    typedef unsigned short uint16_t;
    typedef signed short int16_t;
    typedef unsigned int uint32_t;
    typedef signed int int32_t;
    typedef unsigned long long uint64_t;
    typedef signed long long int64_t;
};


namespace pdal
{


enum EndianType
{
    Endian_Little,
    Endian_Big,
    Endian_Unknown = 128
};

enum PointCountType
{
    PointCount_Fixed,       // getNumPoints will return value (which might be zero, though)
    PointCount_Unknown      // the stage has an unknown count, and getNumPoints will return 0
};

enum StageIteratorType
{
    StageIterator_Sequential = 1,
    StageIterator_Random = 2,
    StageIterator_Block = 4,
    StageIterator_Unknown = 8
};


template <typename T>
class Range
{
public:
    typedef T value_type;

    Range();
    Range(T minimum, T maximum);
    T getMinimum() const;
    void setMinimum(T value);
    T getMaximum() const;
    void setMaximum(T value);
    bool equal(Range const& other) const;
    bool overlaps(Range const& r) const;
    bool contains(Range const& r) const;
    bool contains(T v) const;
    bool empty(void) const;
    void clip(Range const& r);
    void grow(T v);
    void grow(Range const& r);
    void grow(T lo, T hi);
    T length() const;
};

%rename(Range_double) Range<double>;
%template(Range_double) Range<double>;


template <typename T>
class Vector
{
public:
    typedef T value_type;

    Vector();
    Vector(T v0);
    Vector(T v0, T v1);
    Vector(T v0, T v1, T v2);
    Vector(std::vector<T> v);
    T get(std::size_t index);
    void set(std::size_t index, T v);
    void set(std::vector<T> v);
    bool equal(Vector const& other) const;
    std::size_t size() const;
};

%rename(Vector_double) Vector<double>;
%template(Vector_double) Vector<double>;


template <typename T>
class Bounds
{
public:
    typedef typename std::vector< Range<T> > RangeVector;

    Bounds( T minx,
            T miny,
            T minz,
            T maxx,
            T maxy,
            T maxz);
    Bounds(const Vector<T>& minimum, const Vector<T>& maximum);
    T getMinimum(std::size_t const& index) const;
    void setMinimum(std::size_t const& index, T v);
    T getMaximum(std::size_t const& index) const;
    void setMaximum(std::size_t const& index, T v);
    Vector<T> getMinimum();
    Vector<T> getMaximum();
    RangeVector const& dimensions() const;
    std::size_t size() const;
    bool equal(Bounds<T> const& other) const;
    bool overlaps(Bounds const& other) const;
    bool contains(Vector<T> point) const;
    bool contains(Bounds<T> const& other) const;
    void clip(Bounds const& r);
    void grow(Bounds const& r);
    void grow(Vector<T> const& point);
    T volume() const;
    bool empty() const;
};

%rename(Bounds_double) Bounds<double>;
%template(Bounds_double) Bounds<double>;


class SpatialReference
{
public:
    enum WKTModeFlag
    {
        eHorizontalOnly = 1,
        eCompoundOK = 2
    };

    SpatialReference();
    SpatialReference(const std::string& userInput);
    bool empty() const;
    std::string getWKT(WKTModeFlag mode_flag, bool pretty) const;
    void setWKT(std::string const& v);
    void setFromUserInput(std::string const& v);
    std::string getProj4() const;
    void setProj4(std::string const& v);
};



class Dimension
{
public:
   enum Field
    {
        Field_INVALID = 0,
        Field_X,
        Field_Y,
        Field_Z,
        Field_Intensity,
        Field_ReturnNumber,
        Field_NumberOfReturns,
        Field_ScanDirectionFlag,
        Field_EdgeOfFlightLine,
        Field_Classification,
        Field_ScanAngleRank,
        Field_UserData,
        Field_PointSourceId,
        Field_Time,
        Field_Red,
        Field_Green,
        Field_Blue,
        Field_WavePacketDescriptorIndex,
        Field_WaveformDataOffset,
        Field_ReturnPointWaveformLocation,
        Field_WaveformXt,
        Field_WaveformYt,
        Field_WaveformZt,
        Field_Alpha,
        // ...

        // add more here
        Field_User1 = 512,
        Field_User2,
        Field_User3,
        Field_User4,
        Field_User5,
        Field_User6,
        Field_User7,
        Field_User8,
        Field_User9,
        Field_User10,
        Field_User11,
        Field_User12,
        Field_User13,
        Field_User14,
        Field_User15,
        // ...
        // feel free to use your own int here

        Field_LAST = 1023
    };

    enum DataType
    {
        Int8,
        Uint8,
        Int16,
        Uint16,
        Int32,
        Uint32,
        Int64,
        Uint64,
        Float,       // 32 bits
        Double,       // 64 bits
        Undefined
    };

public:
    Dimension(Field field, DataType type);

    std::string const& getFieldName() const;
    Field getField() const;
    DataType getDataType() const;
    static std::string getDataTypeName(DataType);
    static DataType getDataTypeFromString(const std::string&);
    static std::size_t getDataTypeSize(DataType);
    static bool getDataTypeIsNumeric(DataType);
    static bool getDataTypeIsSigned(DataType);
    static bool getDataTypeIsInteger(DataType);
    static std::string const& getFieldName(Field);
    std::size_t getByteSize() const;
    inline std::string getDescription() const;
    inline void setDescription(std::string const& v);
    inline bool isNumeric() const;
    inline bool isSigned() const;
    inline bool isInteger() const;
    inline double getMinimum() const;
    inline void setMinimum(double min);
    inline double getMaximum() const;
    inline void setMaximum(double max);
    inline double getNumericScale() const;
    inline void setNumericScale(double v);
    inline double getNumericOffset() const;
    inline void setNumericOffset(double v);

    template<class T>
    double applyScaling(T x) const;

    inline bool isFinitePrecision() const;
    inline void isFinitePrecision(bool v);
};

%extend Dimension
{
    %template(applyScaling_Int32) applyScaling<boost::int32_t>;
};


class DimensionLayout
{
public:
    DimensionLayout(const pdal::Dimension&);
    const Dimension& getDimension() const;
    inline std::size_t getByteOffset() const;
    inline void setByteOffset(std::size_t v);
    inline std::size_t getPosition() const;
    inline void setPosition(std::size_t v);
};

class Schema
{
public:
    Schema();
    const Dimension& getDimension(std::size_t index) const;
    const std::vector<Dimension>& getDimensions() const;
    int getDimensionIndex(Dimension::Field field, Dimension::DataType datatype) const;
    int getDimensionIndex(const Dimension& dim) const;
    bool hasDimension(Dimension::Field field, Dimension::DataType datatype) const;
    bool hasDimension(const Dimension& dim) const;
};

class SchemaLayout
{
public:
    SchemaLayout(const Schema&);
    const Schema& getSchema() const;
    std::size_t getByteSize() const;
    const DimensionLayout& getDimensionLayout(std::size_t index) const;
};


class PointBuffer
{
public:
    PointBuffer(const SchemaLayout&, boost::uint32_t capacity);
    const Bounds<double>& getSpatialBounds() const;
    void setSpatialBounds(const Bounds<double>& bounds);
    boost::uint32_t getNumPoints() const;
    void setNumPoints(boost::uint32_t v);
    boost::uint32_t& getNumPointsRef();
    boost::uint32_t getCapacity() const;
    const SchemaLayout& getSchemaLayout() const;
    const Schema& getSchema() const;
    //SchemaLayout& getSchemaLayout();
    //Schema& getSchema();
    template<class T> T getField(std::size_t pointIndex, boost::int32_t fieldIndex) const;
    template<class T> void setField(std::size_t pointIndex, boost::int32_t fieldIndex, T value);
    void setFieldData(std::size_t pointIndex, boost::int32_t fieldIndex, const boost::uint8_t* data);
    void copyPointFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer);
    void copyPointsFast(std::size_t destPointIndex, std::size_t srcPointIndex, const PointBuffer& srcPointBuffer, std::size_t numPoints);
    boost::uint64_t getBufferByteLength() const;
    boost::uint64_t getBufferByteCapacity() const;
    //boost::uint8_t* getData(std::size_t pointIndex) const;
    boost::uint8_t* getData(std::size_t pointIndex);
    void setData(boost::uint8_t* data, std::size_t pointIndex);
    void setAllData(boost::uint8_t* data, boost::uint32_t byteCount);
    void getData(boost::uint8_t** data, std::size_t* array_size) const;
};

%extend PointBuffer
{
    %template(getField_Int8) getField<boost::int8_t>;
    %template(getField_UInt8) getField<boost::uint8_t>;
    %template(getField_Int16) getField<boost::int16_t>;
    %template(getField_UInt16) getField<boost::uint16_t>;
    %template(getField_Int32) getField<boost::int32_t>;
    %template(getField_UInt32) getField<boost::uint32_t>;
    %template(getField_Int64) getField<boost::int64_t>;
    %template(getField_UInt64) getField<boost::uint64_t>;
    %template(getField_Float) getField<float>;
    %template(getField_Double) getField<double>;
};


class Option
{
public:
    Option();
    void setName(const std::string& name);
    const std::string& getName() const;
    void setDescription(const std::string& description);
    const std::string& getDescription() const;
   
    template<typename T> T getValue() const;
    template<typename T> void setValue(const T& value);
};

class Options
{
public:
    Options();
    void add(const Option& option);
    const Option& getOption(const std::string & name) const;
    template<typename T> T getValueOrThrow(std::string const& name) const;
    template<typename T> T getValueOrDefault(std::string const& name, T defaultValue) const;
    bool hasOption(std::string const& name) const;
};



class StageBase
{
public:
    virtual void initialize();
    bool isInitialized() const;

    const Options& getOptions() const;

    bool isDebug() const;
    
    bool isVerbose() const;
    boost::uint32_t getVerboseLevel() const; 

    boost::uint32_t getId() const;
protected:
    StageBase(const Options& options);
};

class Stage;

class StageIterator
{
public:
    const Stage& getStage() const;
    boost::uint32_t read(PointBuffer& buffer);
    void readBegin();
    void readBufferBegin(PointBuffer&);
    boost::uint32_t readBuffer(PointBuffer&);
    void readBufferEnd(PointBuffer&);
    void readEnd();
    boost::uint64_t getIndex() const;
    void setChunkSize(boost::uint32_t size);
    boost::uint32_t getChunkSize() const;
protected:
    StageIterator(const Stage& stage);
};


class StageSequentialIterator : public StageIterator
{
public:
    boost::uint64_t skip(boost::uint64_t count);
    bool atEnd() const;
};

class StageRandomIterator : public StageIterator
{
public:
    boost::uint64_t seek(boost::uint64_t position);
};

class StageBlockIterator : public StageIterator
{
public:
};

class Stage : public StageBase
{
public:
    virtual void initialize();

    const Schema& getSchema() const;
    virtual boost::uint64_t getNumPoints() const;
    PointCountType getPointCountType() const;
    const Bounds<double>& getBounds() const;
    const SpatialReference& getSpatialReference() const;
    
    virtual bool supportsIterator (StageIteratorType) const = 0;

    virtual StageSequentialIterator* createSequentialIterator() const;
    virtual StageRandomIterator* createRandomIterator() const;
    virtual StageBlockIterator* createBlockIterator() const;
protected:
    Stage(const Options& options);
};


class Reader : public Stage
{
public:
    virtual void initialize();
protected:
    Reader(const Options& options);
};

class Filter : public Stage
{
public:
    virtual void initialize();
    const Stage& getPrevStage() const;
};

class MultiFilter : public Stage
{
public:
    virtual void initialize();
    //////////////const std::vector<const Stage*> getPrevStages() const;
};

class Writer : public StageBase
{
public:
    virtual void initialize();

    void setChunkSize(boost::uint32_t);
    boost::uint32_t getChunkSize() const;

    boost::uint64_t write(boost::uint64_t targetNumPointsToWrite=0);

    virtual boost::property_tree::ptree serializePipeline() const;

    const Stage& getPrevStage() const;

    const SpatialReference& getSpatialReference() const;
    void setSpatialReference(const SpatialReference&);

protected:
    Writer(Stage& prevStage, const Options& options);
};


%rename(LasReader) drivers::las::Reader;
%rename(LasWriter) drivers::las::Writer;

namespace drivers { namespace las {


enum PointFormat
{
    PointFormat0 = 0,         // base
    PointFormat1 = 1,         // base + time
    PointFormat2 = 2,         // base + color
    PointFormat3 = 3,         // base + time + color
    PointFormat4 = 4,         // base + time + wave
    PointFormat5 = 5,         // base + time + color + wave  (NOT SUPPORTED)
    PointFormatUnknown = 99
};

class VariableLengthRecord
{
public:
    VariableLengthRecord(boost::uint16_t reserved,
                         std::string userId,
                         boost::uint16_t recordId,
                         std::string description,
                         const boost::uint8_t* bytes,
                         std::size_t len);
};



class ReaderBase: public pdal::Reader
{
public:
    virtual PointFormat getPointFormat() const = 0;
    virtual boost::uint8_t getVersionMajor() const = 0;
    virtual boost::uint8_t getVersionMinor() const = 0;
protected:
    ReaderBase(const Options&);
};



class Reader : public ReaderBase
{
public:
    Reader(const Options&);
    Reader(const std::string& filename);
    
    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    const std::string& getFileName() const;

    bool supportsIterator (StageIteratorType t) const;

    StageSequentialIterator* createSequentialIterator() const;
    StageRandomIterator* createRandomIterator() const;

    PointFormat getPointFormat() const;
    boost::uint8_t getVersionMajor() const;
    boost::uint8_t getVersionMinor() const;

    boost::uint64_t getPointDataOffset() const;

    const std::vector<VariableLengthRecord>& getVLRs() const;

    bool isCompressed() const;
};


class Writer : public pdal::Writer
{
public:
    Writer(Stage& prevStage, const Options&);
    Writer(Stage& prevStage, std::ostream*);
    ~Writer();

    virtual void initialize();
    virtual const Options getDefaultOptions() const;

    void setFormatVersion(boost::uint8_t majorVersion, boost::uint8_t minorVersion);
    void setPointFormat(PointFormat);
    void setDate(boost::uint16_t dayOfYear, boost::uint16_t year);
    
    void setProjectId(const boost::uuids::uuid&);

    // up to 32 chars (default is "PDAL")
    void setSystemIdentifier(const std::string& systemId); 
    
    // up to 32 chars (default is "PDAL x.y.z")
    void setGeneratingSoftware(const std::string& softwareId);
    
    void setHeaderPadding(boost::uint32_t const& v);

    // default false
    void setCompressed(bool);
};


}; }; // namespace


// %feature("notabstract") LiblasReader;

//class Utils
//{
//public:
//    static std::istream* openFile(std::string const& filename, bool asBinary=true);
//    static void closeFile(std::istream* ifs);
//};


}; // namespace
