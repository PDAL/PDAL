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

#ifndef LEPCC_API_INCLUDE_GUARD
#define LEPCC_API_INCLUDE_GUARD

#ifndef LEPCC_EXPORT
#define LEPCC_EXPORT
#endif


// all version info in one place;
// increment this if any of the module versions increments:
#define VERSION_LEPCC 1

// LEPCC module versions:
#define VERSION_LEPCC_XYZ 1
#define VERSION_LEPCC_RGB 1
#define VERSION_LEPCC_INTENSITY 1
#define VERSION_LEPCC_FLAGBYTES 1


#ifdef __cplusplus
extern "C" {
#endif

  //! C-API for LEPCC library
  //! Unless specified:
  //!   - all output buffers must have been allocated by caller. 
  //!   - Input buffers do not need to outlive lepcc function call (lepcc will make internal copies if needed).
  

  typedef void* lepcc_ContextHdl;
  typedef unsigned int lepcc_status;
  typedef unsigned int lepcc_blobType;

  //! Create a context ( needed to call lepcc_computeCompressedSizeXXX() and encode/decode functions )
  LEPCC_EXPORT lepcc_ContextHdl lepcc_createContext();

  //! Free the resource associated with the context. 
  LEPCC_EXPORT void lepcc_deleteContext(lepcc_ContextHdl* ctx);

  //! Compute the size in bytes required to compress the input xyz coordinates. 
  //! maxXErr is the max compression error tolerated per point in x. Same for y and z. 
  //! The points get resorted while compressed. orderOut has that order (the orig index for each point). 
  //! Needed to sort RGB or Intensity values to point order. 

  LEPCC_EXPORT lepcc_status lepcc_computeCompressedSizeXYZ(lepcc_ContextHdl ctx, unsigned int nPts, const double* xyzArray,
    double maxXErr, double maxYErr, double maxZErr, unsigned int* nBytesOut, unsigned int* orderOut = 0);

  //! Compute the size in bytes required to compress the input RGB values.
  LEPCC_EXPORT lepcc_status lepcc_computeCompressedSizeRGB(lepcc_ContextHdl ctx, unsigned int nRGB, const unsigned char* rgbArray, unsigned int* nBytesOut);

  //! Compute the size in bytes required to compress the input intensity values (16 bit). 
  LEPCC_EXPORT lepcc_status lepcc_computeCompressedSizeIntensity(lepcc_ContextHdl ctx, unsigned int nValues, const unsigned short* valArray, unsigned int* nBytesOut);

  //! Compute the size in bytes required to compress the input flag bytes. 
  LEPCC_EXPORT lepcc_status lepcc_computeCompressedSizeFlagBytes(lepcc_ContextHdl ctx, unsigned int nValues, const unsigned char* valArray, unsigned int* nBytesOut);



  //! Encode xyz coordinates following a call to lepcc_computeCompressedSizeXYZ(). Output buffers must have been sized accordingly. 
  LEPCC_EXPORT lepcc_status lepcc_encodeXYZ(lepcc_ContextHdl ctx, unsigned char** ppByteOut, int bufferSizeOut);

  //! Encode RGB following a call to lepcc_computeCompressedSizeRGB(). Output buffers must have been sized accordingly.
  LEPCC_EXPORT lepcc_status lepcc_encodeRGB(lepcc_ContextHdl ctx, unsigned char** ppByteOut, int bufferSizeOut);

  //! Encode Intensity following a call to lepcc_computeCompressedSizeIntensity(). Output buffers must have been sized accordingly.
  LEPCC_EXPORT lepcc_status lepcc_encodeIntensity(lepcc_ContextHdl ctx, unsigned char** ppByteOut, int bufferSizeOut, const unsigned short* intensityArray, unsigned int nValues);

  //! Encode flag bytes following a call to lepcc_computeCompressedSizeFlagBytes(). Output buffers must have been sized accordingly.
  LEPCC_EXPORT lepcc_status lepcc_encodeFlagBytes(lepcc_ContextHdl ctx, unsigned char** ppByteOut, int bufferSizeOut, const unsigned char* flagByteArray, unsigned int nValues);


  //! Use the following to find out what type of blob is next in the compressed stream, and what size it is
  LEPCC_EXPORT int lepcc_getBlobInfoSize();
  LEPCC_EXPORT lepcc_status lepcc_getBlobInfo(lepcc_ContextHdl ctx, const unsigned char* packed, int bufferSize, lepcc_blobType* blobType, unsigned int* blobSize);


  //! get the number of xyz coordinates available in the encoded buffer
  LEPCC_EXPORT lepcc_status lepcc_getPointCount(lepcc_ContextHdl ctx, const unsigned char* packed, int bufferSize, unsigned int* countOut);

  //! get the number of RGB values available in the encoded buffer
  LEPCC_EXPORT lepcc_status lepcc_getRGBCount(lepcc_ContextHdl ctx, const unsigned char* packed, int bufferSize, unsigned int* countOut);

  //! get the number of intensity values available in the encoded buffer
  LEPCC_EXPORT lepcc_status lepcc_getIntensityCount(lepcc_ContextHdl ctx, const unsigned char* packed, int bufferSize, unsigned int* countOut);

  //! get the number of flag byte values available in the encoded buffer
  LEPCC_EXPORT lepcc_status lepcc_getFlagByteCount(lepcc_ContextHdl ctx, const unsigned char* packed, int bufferSize, unsigned int* countOut);



  //! Decode the xyz coordinates. Output buffer must have the size returned by lepcc_getPointCount() * 3
  LEPCC_EXPORT lepcc_status lepcc_decodeXYZ(lepcc_ContextHdl ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nPtsInOut, double* xyzBuffOut);

  //! Decode the RGB colors. Output buffer must have the size returned by lepcc_getRGBCount() * 3
  LEPCC_EXPORT lepcc_status lepcc_decodeRGB(lepcc_ContextHdl ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nRGBInOut, unsigned char* rbgBuffOut);

  //! Decode the intensities. output buffer must have the size returned by lepcc_getIntensityCount()
  LEPCC_EXPORT lepcc_status lepcc_decodeIntensity(lepcc_ContextHdl ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nValues, unsigned short* intensityBuffOut);

  //! Decode the flag bytes. output buffer must have the size returned by lepcc_getFlagByteCount()
  LEPCC_EXPORT lepcc_status lepcc_decodeFlagBytes(lepcc_ContextHdl ctx, const unsigned char** ppByte, int bufferSize, unsigned int* nValues, unsigned char* flagBytesBuffOut);


#ifdef __cplusplus
}
#endif


#endif //LEPCC_API_INCLUDE_GUARD
