/******************************************************************************
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  swig/C# bindings for liblas
 * Author:   Michael P. Gerlek (mpg@flaxen.com)
 *
 ******************************************************************************
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
 *     * Neither the name of the Martin Isenburg or Iowa Department 
 *       of Natural Resources nor the names of its contributors may be 
 *       used to endorse or promote products derived from this software 
 *       without specific prior written permission.
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
 
%module libpc_swig_cpp

%{
#include "libpc/libpc_config.hpp"
#include "libpc/Dimension.hpp"
#include "libpc/Header.hpp"
#include "libpc/Schema.hpp"
#include "libpc/Reader.hpp"
#include "libpc/../../src/drivers/liblas/reader.hpp"
#include "libpc/../../src/drivers/liblas/header.hpp"
%}

%include "typemaps.i"

// C# support for std::string
%include "std_string.i"

// C# support for std::vector<T>
%include "std_vector.i"
namespace std {
   %template(VectorU8) vector<unsigned char>;
   %template(VectorU32) vector<unsigned int>;
};
 

// fix up some missing types
namespace std
{
    typedef unsigned int size_t;
};

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

// BUG: how do I do a rename such that "SWIGTYPE_p_std__istream" can
// become something like "IStreamHandle"?

namespace libpc
{

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
        Field_GpsTime,
        Field_Red,
        Field_Green,
        Field_Blue,
        Field_WavePacketDescriptorIndex,
        Field_WaveformDataOffset,
        Field_ReturnPointWaveformLocation,
        Field_WaveformXt,
        Field_WaveformYt,
        Field_WaveformZt,
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
        Double       // 64 bits
    };

public:
    Dimension(Field field, DataType type);
    static std::string getDataTypeName(DataType);
};



class LiblasReader
{
public:
    LiblasReader(std::istream&);
    const libpc::LiblasHeader& getLiblasHeader() const;
    boost::int8_t getPointFormatNumber() const;
};


class Stage
{
public:
    Stage();
    virtual const std::string& getName() const = 0;
    boost::uint32_t read(PointData&);
    virtual void readBegin(boost::uint32_t numPointsToRead) = 0;
    virtual boost::uint32_t readBuffer(PointData&) = 0;
    virtual void readEnd(boost::uint32_t numPointsRead) = 0;
    virtual void seekToPoint(boost::uint64_t pointNum) = 0;
    virtual boost::uint64_t getCurrentPointIndex() const = 0;
    boost::uint64_t getNumPoints() const;
    bool atEnd() const;
    const Header& getHeader() const;
};


class Reader : public Stage
{
public:
    Reader();

    virtual void readBegin(boost::uint32_t numPointsToRead);

    virtual void readEnd(boost::uint32_t numPointsRead);

    virtual void seekToPoint(boost::uint64_t pointNum);

    virtual boost::uint64_t getCurrentPointIndex() const;
};


class LiblasHeader : public Reader
{
public:
    LiblasHeader();
};



class Utils
{
public:
    static std::istream* openFile(std::string const& filename, bool asBinary=true);
    static void closeFile(std::istream* ifs);
};




}; // namespace
