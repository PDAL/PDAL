/*
Copyright 2016 Esri

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.

A local copy of the license and additional notices are located with the
source distribution at:

http://github.com/Esri/lepcc/

Contributors:  Ronald Poirrier, Thomas Maurer
*/

#pragma once
#include "lepcc_c_api.h"
#include "lepcc_types.h"  // only for ErrCode 


namespace lepcc
{

//! Template API for LEPCC. 
//! This API allows callers to:
//!       - Use containers of their choosing for input/output without resorting to low-level C API. 
//!       - Avoid C++ STL runtime dependencies at the API level. 
//! 
/*!
//=== Example: ===
std::vector< Point3D > points( 24 );
// ... set the coords
std::string packedBuffer; //could  have been std::vector< char > anotherBuffer;
auto hr = encodeXYZ( points, &packedBuffer );
// ... check for error
std::vector< Point3D > readback;
hr = decodeXYZ( packedBuffer, &readback);
//... check for error

*/


template< class ByteVec_t>
inline ErrCode getBlobType(const ByteVec_t& packed, BlobType* type);

//!helper template to get the number of coordinates encoded in the provided buffer. 
template< class ByteVec_t >
inline ErrCode    getXyzCount(const ByteVec_t& packed, unsigned int* countOut);

//!helper template to get the number of RGB encoded in the provided buffer. 
template< class ByteVec_t >
inline ErrCode    getRgbCount(const ByteVec_t& packed, unsigned int* countOut);


//! Helper template to encode XYZ 
template< class Double3Vec_t, class ByteVec_t, class IntVec_t >
inline ErrCode     encodeXYZ( const Double3Vec_t& in, ByteVec_t* out, double errX, double errY, double errZ, IntVec_t* order=nullptr);


//! Helper template to decode XYZ 
template< class Double3Vec_t, class ByteVec_t>
inline ErrCode     decodeXYZ(const ByteVec_t& in, Double3Vec_t* out);

//! Helper template to encode RGB colors using clustering: 
template< class RgbVec_t, class ByteVec_t >
inline ErrCode     encodeRGB(const RgbVec_t& in, ByteVec_t* out);

//! Helper template to decode RGB colors:
template< class RgbVec_t, class ByteVec_t>
inline ErrCode     decodeRGB(const ByteVec_t& in, RgbVec_t* out);

//! Helper template to encode Intensity colors using clustering: 
template< class UInt16Vec_t, class ByteVec_t >
inline ErrCode     encodeIntensity(const UInt16Vec_t& in, ByteVec_t* out);

//! Helper template to decode RGB colors:
template< class UInt16Vec_t, class ByteVec_t>
inline ErrCode     decodeIntensity(const ByteVec_t& in, UInt16Vec_t* out);



//============================ inline definition: ========================




//RAII helper class so that we don't leak a context.
class CtxGuard
{
public:
  CtxGuard() : m_ctx(lepcc_createContext()) {}
  ~CtxGuard() {
    if (m_ctx)
    {
      lepcc_ContextHdl copy = m_ctx;
      lepcc_deleteContext(&copy);
    }
  }
  operator lepcc_ContextHdl() { return m_ctx;  }
private:
  lepcc_ContextHdl const m_ctx; // DISALLOW copy copy-ctor /assign op.
};


template< class ByteVec_t>
inline ErrCode getBlobType(const ByteVec_t& packed, BlobType* type)
{
  static_assert(sizeof(packed[0]) == sizeof(char), "Input must be a byte array");
  //create the context 
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  unsigned int  blobSize;
  return (ErrCode)lepcc_getBlobInfo(ctx, reinterpret_cast< const unsigned char*>(&packed[0]), (int)packed.size(), (lepcc_blobType*)(type),  &blobSize );
}

template< class ByteVec_t >
inline ErrCode    getXyzCount(const ByteVec_t& packed, unsigned int* countOut)
{
  static_assert(sizeof(packed[0]) == sizeof(char), "Input must be a byte array");
  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  return (ErrCode)lepcc_getPointCount(ctx, reinterpret_cast< const unsigned char*>(&packed[0]), (int)packed.size(), countOut);
}


template< class ByteVec_t >
inline ErrCode    getRgbCount(const ByteVec_t& packed, unsigned int* countOut)
{
  static_assert(sizeof(packed[0]) == sizeof(char), "Input must be a byte array");
  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  return (ErrCode)lepcc_getRGBCount(ctx, reinterpret_cast<const unsigned char*>(&(packed)[0]), (int)packed.size(), countOut);
}


template< class Double3Vec_t, class ByteVec_t, class IntVec_t >
inline ErrCode     encodeXYZ(const Double3Vec_t& in, ByteVec_t* out, double errX, double errY, double errZ, IntVec_t* orderOut)
{
  //sanity:
  static_assert(sizeof(in[0]) == sizeof(double) * 3, "Input XYZ array must contain 3 doubles per point");
  static_assert(sizeof((*out)[0]) == sizeof(char), "Output array must be a byte array");
  static_assert(sizeof((*orderOut)[0]) == sizeof(int), "Order array must be 4 byte per element");

  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get the size:
  unsigned int nPackBytes=0;
  if (orderOut)
    orderOut->resize(in.size());
  auto hr = (ErrCode)lepcc_computeCompressedSizeXYZ(ctx, (uint32)in.size(), reinterpret_cast<const double*>(in.data()), errX, errY, errZ, &nPackBytes
                                                    , orderOut ? reinterpret_cast<unsigned int*>( orderOut->data() ) : nullptr);
  if (hr != ErrCode::Ok || nPackBytes == 0)
    return hr;
  //ready to pack:
  out->resize( nPackBytes );
  unsigned char* outPtr = reinterpret_cast< unsigned char*> (&(*out)[0]);
  //const double* inPtr = reinterpret_cast< const double*> (&in[0]);
  hr = (ErrCode)lepcc_encodeXYZ(ctx, &outPtr, nPackBytes);
  if (hr != ErrCode::Ok)
    return hr;
  //check actual size:
  auto actualSize = outPtr - reinterpret_cast< unsigned char*>( &(*out)[0] );
  out->resize(actualSize);
  return hr;
}



template< class Double3Vec_t, class ByteVec_t>
inline ErrCode     decodeXYZ(const ByteVec_t& in, Double3Vec_t* out)
{
  //sanity:
  static_assert(sizeof((*out)[0]) == sizeof(double) * 3, "Output XYZ array must contain 3 doubles per point");
  static_assert(sizeof(in[0]) == sizeof(char), "Input array must be byte array");

  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get pts count:
  unsigned int nPts=0;
  auto hr = (ErrCode)lepcc_getPointCount(ctx, reinterpret_cast< const unsigned char*>(in.data()), (int)in.size(), &nPts);
  if (hr != ErrCode::Ok )
    return hr;
  out->resize(nPts);
  if (!out->size())
    return hr; //done.

  //ready to unpack:
  const unsigned char* inPtr = reinterpret_cast< const unsigned char*> (&in[0] );
  double* outPtr = reinterpret_cast< double*> (&(*out)[0]);
  return (ErrCode)lepcc_decodeXYZ(ctx, &inPtr, (unsigned int)in.size(), &nPts, outPtr );
}


template< class RgbVec_t, class ByteVec_t >
inline ErrCode     encodeRGB(const RgbVec_t& in, ByteVec_t* out)
{
  //sanity:
  static_assert(sizeof(in[0]) == sizeof(char) * 3, "Input RGB array must contain 3 byte per color");
  static_assert(sizeof((*out)[0]) == sizeof(char), "Output array must be a byte array");
  
  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get the size:
  unsigned int nPackBytes = 0;
  auto hr = (ErrCode)lepcc_computeCompressedSizeRGB(ctx, (uint32)in.size(), reinterpret_cast<const unsigned char*>(&in[0]), &nPackBytes);
  if (hr != ErrCode::Ok || nPackBytes == 0)
    return hr;
  //ready to pack:
  out->resize(nPackBytes);
  unsigned char* outPtr = reinterpret_cast< unsigned char*> (&(*out)[0]);
  //const unsigned char* inPtr = reinterpret_cast< const unsigned char*> (&in[0]);
  hr = (ErrCode)lepcc_encodeRGB(ctx,  &outPtr, nPackBytes);
  if (hr != ErrCode::Ok)
    return hr;
  //check actual size:
  auto actualSize = outPtr - reinterpret_cast< unsigned char*>(&(*out)[0]);
  out->resize(actualSize);
  return hr;
}

template< class RgbVec_t, class ByteVec_t>
inline ErrCode     decodeRGB(const ByteVec_t& in, RgbVec_t* out)
{
  //sanity:
  static_assert(sizeof((*out)[0]) == sizeof(char) * 3, "Output RGB array must contain 3 byte per Color");
  static_assert(sizeof(in[0]) == sizeof(char), "Input array must be byte array");

  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get color count:
  unsigned int nColors = 0;
  auto hr = (ErrCode)lepcc_getRGBCount(ctx, reinterpret_cast< const unsigned char*>( in.data() ), (int)in.size(), &nColors);
  if (hr != ErrCode::Ok)
    return hr;
  out->resize(nColors);
  if (!out->size())
    return hr; //done.

  //ready to unpack:
  const unsigned char* inPtr = reinterpret_cast< const unsigned char*> (&in[0]);
  unsigned char* outPtr = reinterpret_cast< unsigned char*> (&(*out)[0]);
  return (ErrCode)lepcc_decodeRGB(ctx, &inPtr, (unsigned int)in.size(), &nColors, outPtr);
}

template< class Uint16Vec_t, class ByteVec_t >
inline ErrCode     encodeIntensity(const Uint16Vec_t& in, ByteVec_t* out)
{
  //sanity:
  static_assert(sizeof(in[0]) == sizeof(short), "Input Intensity array must be 2 bytes per val");
  static_assert(sizeof((*out)[0]) == sizeof(char), "Output array must be a byte array");

  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get the size:
  unsigned int nPackBytes = 0;
  auto hr = (ErrCode)lepcc_computeCompressedSizeIntensity(ctx, (uint32)in.size(), reinterpret_cast<const unsigned short*>(&in[0]), &nPackBytes);
  if (hr != ErrCode::Ok || nPackBytes == 0)
    return hr;
  //ready to pack:
  out->resize(nPackBytes);
  unsigned char* outPtr = reinterpret_cast< unsigned char*> (&(*out)[0]);
  //const unsigned char* inPtr = reinterpret_cast< const unsigned char*> (&in[0]);
  hr = (ErrCode)lepcc_encodeIntensity(ctx, &outPtr, nPackBytes, reinterpret_cast<const unsigned short*>(&in[0]), (uint32)in.size());
  if (hr != ErrCode::Ok)
    return hr;
  //check actual size:
  auto actualSize = outPtr - reinterpret_cast< unsigned char*>(&(*out)[0]);
  out->resize(actualSize);
  return hr;
}

template< class Uint16Vec_t, class ByteVec_t>
inline ErrCode     decodeIntensity(const ByteVec_t& in, Uint16Vec_t* out)
{
  //sanity:
  static_assert(sizeof((*out)[0]) == sizeof(short), "Output intensity array must  be 2 bytes per val");
  static_assert(sizeof(in[0]) == sizeof(char), "Input array must be byte array");

  //create the context
  CtxGuard ctx;
  if (!ctx)
    return ErrCode::Failed;
  //get intensity count:
  unsigned int nVal = 0;
  auto hr = (ErrCode)lepcc_getIntensityCount(ctx, reinterpret_cast< const unsigned char*>(in.data()), (int)in.size(), &nVal);
  if (hr != ErrCode::Ok)
    return hr;
  out->resize(nVal);
  if (!out->size())
    return hr; //done.

  //ready to unpack:
  const unsigned char* inPtr = reinterpret_cast< const unsigned char*> (&in[0]);
  unsigned short* outPtr = reinterpret_cast< unsigned short*> (&(*out)[0]);
  return (ErrCode)lepcc_decodeIntensity(ctx, &inPtr, (unsigned int)in.size(), &nVal, outPtr);
}



} //endof ::lepcc

