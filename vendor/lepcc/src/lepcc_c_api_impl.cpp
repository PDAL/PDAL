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

#include "lepcc_c_api.h"
#include "LEPCC.h"
#include "ClusterRGB.h"
#include "Intensity.h"
#include "FlagBytes.h"

using namespace lepcc;

struct CtxImpl
{
  CtxImpl() : xyz(nullptr), rgb(nullptr), inty(nullptr), flags(nullptr) {}
  ~CtxImpl() { delete xyz; delete rgb; delete inty; delete flags; }

  LEPCC*      xyz;
  ClusterRGB* rgb;
  Intensity*  inty;
  FlagBytes*  flags;

private:
  CtxImpl(const CtxImpl&);  // disallow
  CtxImpl& operator=(const CtxImpl&);  // disallow
};

lepcc_ContextHdl lepcc_createContext()
{
  return reinterpret_cast<lepcc_ContextHdl>(new CtxImpl());
}

void lepcc_deleteContext(lepcc_ContextHdl* ctx)
{
  delete reinterpret_cast<CtxImpl*>(*ctx);
  *ctx = nullptr;
}

// ---- XYZ: 

lepcc_status lepcc_computeCompressedSizeXYZ(lepcc_ContextHdl _ctx, unsigned int nPts, const double* xyzArray, 
  double maxXErr, double maxYErr, double maxZErr, unsigned int* nBytes, unsigned int* orderOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !nBytes || !xyzArray)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->xyz)
    ctx->xyz = new LEPCC();

  int64 nBytes64 = 0;
  auto hr = ctx->xyz->ComputeNumBytesNeededToEncode(nPts, reinterpret_cast<const Point3D*>(xyzArray), maxXErr, maxYErr, maxZErr, nBytes64);
  if (hr != ErrCode::Ok )
      return (lepcc_status)hr;

  *nBytes = (uint32)nBytes64;

  if (orderOut && !ctx->xyz->GetOrigPointIndexes(orderOut, nPts))
    return (lepcc_status)ErrCode::Failed;

  return (lepcc_status)hr;
}

lepcc_status lepcc_encodeXYZ(lepcc_ContextHdl _ctx, unsigned char** ppByteOut, int bufferSizeOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !ctx->xyz)
    return (lepcc_status)ErrCode::WrongParam;
 
  auto hr = ctx->xyz->Encode(ppByteOut, int64(bufferSizeOut));
  return (lepcc_status)hr;
}

lepcc_status lepcc_getPointCount(lepcc_ContextHdl , const unsigned char* packed, int bufferSize, unsigned int* count)
{
  return (lepcc_status)LEPCC::GetNumPointsFromHeader(packed, (int64)bufferSize, *count);
}

lepcc_status lepcc_decodeXYZ(lepcc_ContextHdl _ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nPtsInOut, double* xyzBuffOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if(!ctx)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->xyz)
    ctx->xyz = new LEPCC();

  return (lepcc_status)ctx->xyz->Decode(ppByte, bufferSize, *nPtsInOut, reinterpret_cast<Point3D*>(xyzBuffOut));
}

// ---- RGB: 

lepcc_status lepcc_computeCompressedSizeRGB(lepcc_ContextHdl _ctx, unsigned int nRGB, const unsigned char* rgbArray, unsigned int* nBytes)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !nBytes || !rgbArray)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->rgb)
    ctx->rgb = new ClusterRGB();

  int64 nBytes64 = 0;
  auto hr = ctx->rgb->ComputeNumBytesNeededToEncode(nRGB, reinterpret_cast<const RGB_t*>(rgbArray), nBytes64);
  if (hr != ErrCode::Ok)
    return (lepcc_status)hr;

  *nBytes = (uint32)nBytes64;

  return (lepcc_status)hr;
}

lepcc_status lepcc_encodeRGB(lepcc_ContextHdl _ctx, unsigned char** ppByteOut, int bufferSizeOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !ctx->rgb)
    return (lepcc_status)ErrCode::WrongParam;

  auto hr = ctx->rgb->Encode(ppByteOut, int64(bufferSizeOut));
  return (lepcc_status)hr;
}

lepcc_status lepcc_getRGBCount(lepcc_ContextHdl , const unsigned char* packed, int bufferSize, unsigned int* count)
{
  return (lepcc_status)ClusterRGB::GetNumPointsFromHeader(packed, (int64)bufferSize, *count);
}

lepcc_status lepcc_decodeRGB(lepcc_ContextHdl _ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nRGBInOut, unsigned char* rbgBuffOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->rgb)
    ctx->rgb= new ClusterRGB();

  return (lepcc_status)ctx->rgb->Decode(ppByte, bufferSize, *nRGBInOut, reinterpret_cast<RGB_t*>(rbgBuffOut));
}

// ---- Intensity: 

lepcc_status lepcc_computeCompressedSizeIntensity(lepcc_ContextHdl _ctx, unsigned int nValues, const unsigned short* valArray, unsigned int* nBytes)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !nBytes || !valArray)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->inty)
    ctx->inty = new Intensity();

  int64 nBytes64 = 0;
  auto hr = ctx->inty->ComputeNumBytesNeededToEncode(nValues, valArray, nBytes64);
  if (hr != ErrCode::Ok)
    return (lepcc_status)hr;
  *nBytes = (uint32)nBytes64;
  return (lepcc_status)hr;
}

lepcc_status lepcc_encodeIntensity(lepcc_ContextHdl _ctx, unsigned char** ppByteOut, int bufferSizeOut,  const unsigned short* intensities, unsigned int nElem)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !ctx->inty || !intensities)
    return (lepcc_status)ErrCode::WrongParam;

  auto hr = ctx->inty->Encode(ppByteOut, int64(bufferSizeOut), nElem, intensities);
  return (lepcc_status)hr;
}

lepcc_status lepcc_getIntensityCount(lepcc_ContextHdl , const unsigned char* packed, int bufferSize, unsigned int* count)
{
  return (lepcc_status)Intensity::GetNumPointsFromHeader(packed, (int64)bufferSize, *count);
}

lepcc_status lepcc_decodeIntensity(lepcc_ContextHdl _ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nElemInOut, unsigned short* valBuffOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->inty)
    ctx->inty = new Intensity();

  return (lepcc_status)ctx->inty->Decode(ppByte, bufferSize, *nElemInOut, valBuffOut);
}

// ---- Flag bytes: 

lepcc_status lepcc_computeCompressedSizeFlagBytes(lepcc_ContextHdl _ctx, unsigned int nValues, const unsigned char* valArray, unsigned int* nBytes)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !nBytes || !valArray)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->flags)
    ctx->flags = new FlagBytes();

  int64 nBytes64 = 0;
  auto hr = ctx->flags->ComputeNumBytesNeededToEncode(nValues, valArray, nBytes64);
  if (hr != ErrCode::Ok)
    return (lepcc_status)hr;
  *nBytes = (uint32)nBytes64;
  return (lepcc_status)hr;
}

lepcc_status lepcc_encodeFlagBytes(lepcc_ContextHdl _ctx, unsigned char** ppByteOut, int bufferSizeOut, const unsigned char* flagByteArray, unsigned int nElem)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx || !ctx->flags || !flagByteArray)
    return (lepcc_status)ErrCode::WrongParam;

  auto hr = ctx->flags->Encode(ppByteOut, int64(bufferSizeOut), nElem, flagByteArray);
  return (lepcc_status)hr;
}

lepcc_status lepcc_getFlagByteCount(lepcc_ContextHdl, const unsigned char* packed, int bufferSize, unsigned int* count)
{
  return (lepcc_status)FlagBytes::GetNumPointsFromHeader(packed, (int64)bufferSize, *count);
}

lepcc_status lepcc_decodeFlagBytes(lepcc_ContextHdl _ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nElemInOut, unsigned char* valBuffOut)
{
  CtxImpl* ctx = reinterpret_cast<CtxImpl*>(_ctx);
  if (!ctx)
    return (lepcc_status)ErrCode::WrongParam;

  if (!ctx->flags)
    ctx->flags = new FlagBytes();

  return (lepcc_status)ctx->flags->Decode(ppByte, bufferSize, *nElemInOut, valBuffOut);
}


int lepcc_getBlobInfoSize() { return 16 + 8; }

lepcc_status lepcc_getBlobInfo(lepcc_ContextHdl , const unsigned char* packed, int bufferSize,
  lepcc_blobType* blobType, unsigned int* blobSize)
{
  if (bufferSize < lepcc_getBlobInfoSize() || !packed || !blobType || !blobSize)
    return (lepcc_status)ErrCode::WrongParam;

  if (ErrCode::Ok == LEPCC::GetBlobSize(packed, bufferSize, *blobSize))
    *blobType = (lepcc_blobType)BlobType::bt_XYZ;
  else if (ErrCode::Ok == ClusterRGB::GetBlobSize(packed, bufferSize, *blobSize))
    *blobType = (lepcc_blobType)BlobType::bt_RGB;
  else if (ErrCode::Ok == Intensity::GetBlobSize(packed, bufferSize, *blobSize))
    *blobType = (lepcc_blobType)BlobType::bt_Intensity;
  else if (ErrCode::Ok == FlagBytes::GetBlobSize(packed, bufferSize, *blobSize))
    *blobType = (lepcc_blobType)BlobType::bt_FlagBytes;
  else
    return (lepcc_status)ErrCode::Failed;

  return (lepcc_status)ErrCode::Ok;
}

