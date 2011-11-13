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
 
%module(directors="1", allprotected="1")  pdal_swig_cpp
%include "typemaps.i"
%include "std_string.i"
%include "std_vector.i"
%{
#include <iostream>
#include "pdal/external/boost/uuid/uuid.hpp"
#include "pdal/pdal_config.hpp"

#include "pdal/Bounds.hpp"
#include "pdal/Range.hpp"
#include "pdal/DimensionId.hpp"
#include "pdal/Dimension.hpp"
#include "pdal/Schema.hpp"
#include "pdal/PointBuffer.hpp"
#include "pdal/SpatialReference.hpp"
#include "pdal/StageIterator.hpp"
#include "pdal/Reader.hpp"
#include "pdal/Writer.hpp"
#include "pdal/Filter.hpp"
#include "pdal/MultiFilter.hpp"
#include "pdal/Options.hpp"
#include "pdal/UserCallback.hpp"
#include "pdal/Vector.hpp"

#include "pdal/filters/DecimationFilter.hpp"
#include "pdal/filters/ScalingFilter.hpp"
#include "pdal/drivers/las/Support.hpp"
#include "pdal/drivers/las/VariableLengthRecord.hpp"
#include "pdal/drivers/las/Reader.hpp"
#include "pdal/drivers/las/Writer.hpp"
%}

namespace std
{
   %template(std_vector_u8) vector<unsigned char>;
   %template(std_vector_double) vector<double>;
   %template(std_vector_Dimension) vector<pdal::Dimension>;
   %template(std_vector_Range_double) vector<pdal::Range<double> >;

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


%include "pdal/export.hpp"

# These enums get initialized with an unsigned int which conflicts with the "int" impl. of the enum.
%typemap(csbase) pdal::StageIteratorType "uint"
%typemap(csbase) pdal::StageOperationType "uint"

%include "pdal/types.hpp"
%include "pdal/Vector.hpp"
%rename(Vector_double) pdal::Vector<double>;
%template(Vector_double) pdal::Vector<double>;

%include "pdal/Bounds.hpp"
%rename(Bounds_double) pdal::Bounds<double>;
%template(Bounds_double) pdal::Bounds<double>;

%include "pdal/Range.hpp"
%rename(Range_double) pdal::Range<double>;
%template(Range_double) pdal::Range<double>;

# not yet implemented in pdal.lib
%ignore   pdal::SpatialReference::getDescription;
%ignore   pdal::Options::remove;
%include "pdal/SpatialReference.hpp"
%include "pdal/Options.hpp"
%extend pdal::Option
{
    %template(setValue_String) setValue<std::string>;
};

%include "pdal/DimensionId.hpp"
%include "pdal/Dimension.hpp"
%extend pdal::Dimension
{
    %template(applyScaling_Int32) applyScaling<boost::int32_t>;
};
%include "pdal/Schema.hpp"
%include "pdal/MetadataRecord.hpp"
%include "pdal/StageBase.hpp"
%include "pdal/Stage.hpp"
%include "pdal/StageIterator.hpp"
%include "pdal/Filter.hpp"
%include "pdal/MultiFilter.hpp"
%include "pdal/Reader.hpp"
%include "pdal/Writer.hpp"


%include "pdal/drivers/las/Support.hpp"
%include "pdal/drivers/las/VariableLengthRecord.hpp"

#  Prevent GC from nuking child wrappers passed in to parent wrapper ctor.
%define HOLD_REFERENCE(Class, RefClass)
   %typemap(cscode) Class %{
      private RefClass refTo##RefClass;
      internal void set##RefClass##Ref(RefClass obj) { refTo##RefClass = obj; }
   %}
%enddef
%define PASS_REFERENCE_CTOR(Class, RefClass, Arg)
  %typemap(csconstruct, excode=SWIGEXCODE) Class %{: this($imcall, true) {
      $imcall;set##RefClass##Ref(Arg);$excode
      }
   %}
%enddef

HOLD_REFERENCE(pdal::filters::DescalingFilter, Stage)
PASS_REFERENCE_CTOR(pdal::filters::DescalingFilter, Stage, prevStage)
%include "pdal/filters/ScalingFilter.hpp"

HOLD_REFERENCE( pdal::drivers::las::Writer, Stage)
PASS_REFERENCE_CTOR(pdal::drivers::las::Writer, Stage, prevStage)
%rename(LasWriter) pdal::drivers::las::Writer;

%rename(LasReader) pdal::drivers::las::Reader;
%include "pdal/drivers/las/ReaderBase.hpp"
%include "pdal/drivers/las/Reader.hpp"
%include "pdal/drivers/las/Writer.hpp"
%include "pdal/PointBuffer.hpp"
%extend pdal::PointBuffer
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


%feature("director") pdal::UserCallback;
%include "pdal/UserCallback.hpp"
