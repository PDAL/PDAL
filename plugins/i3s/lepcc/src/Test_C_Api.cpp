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

Contributors:  Thomas Maurer
*/

// Test_C_Api.cpp
//

#include <cstring>
#include <cmath>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>
#include "lepcc_c_api.h"
#include "lepcc_types.h"

using namespace std;
using namespace std::chrono;
using namespace lepcc;

// -------------------------------------------------------------------------- ;

int ReadBlobSize(lepcc_ContextHdl ctx, FILE* fp);

// -------------------------------------------------------------------------- ;

int main(int argc, char* argv[])
{
  double maxXErr = 9e-8, maxYErr = maxXErr;
  double maxZErr = 0.01;

  int mode = 0;    // 0 - computeCompressedSize, 1 - encode, 2 - decode, 3 - test

  bool origHasXYZ = false;
  bool origHasRGB = false;
  bool decodHasRGB = origHasRGB;
  bool origHasIntensity = true;

  string fn, fnOut;

  if (mode != 2)
  {
    //fn = "tilesXYZ.bin";
    //fn = "tilesXYZ_RGB.bin";
    //fn = "heritage_rgb_only.bin";
    fn = "sonoma18_intensity_only.bin";
    fnOut = "tilesCompressed.bin";
  }
  else
  {
    fn = "tilesCompressed.bin";
    fnOut = "tilesUncompressed.bin";
  }

  FILE* fp = 0;
  fopen_s(&fp, fn.c_str(), "rb");
  if (!fp)
  {
    printf("Error in main(): Cannot read from file %s.\n", fn.c_str());  return 0;
  }

  FILE* fpOut = 0;
  if (mode == 1 || mode == 2)    //  || mode == 0)
  {
    fopen_s(&fpOut, fnOut.c_str(), "wb");
    if (!fpOut)
    {
      printf("Error in main(): Cannot write to file %s.\n", fnOut.c_str());  return 0;
    }
    fclose(fpOut);
    fopen_s(&fpOut, fnOut.c_str(), "ab");    // open again in append mode
  }

  double totalNumPoints = 0, totalNumTiles = 0;
  double minPts = 1e16, maxPts = 0;
  double maxDecErrX = 0, maxDecErrY = 0, maxDecErrZ = 0;
  double maxDecErrRGB = 0, totalDecErrRGB = 0;
  double maxDecErrIntensity = 0, totalDecErrIntensity = 0;
  int64 totalNumBytesCompressed = 0;

  lepcc_ContextHdl ctx = lepcc_createContext();
  lepcc_ContextHdl ctxDec = lepcc_createContext();
  lepcc_status hr;

  vector<Point3D> ptVec, decPtVec;
  vector<RGB_t> rgbVec, sortedRgbVec, decRgbVec;
  vector<Byte> byteVec;
  vector<uint16> intensityVec, decIntensityVec;
  vector<uint32> orderVec;

  high_resolution_clock::time_point t0 = high_resolution_clock::now();
  
  if (mode != 2)    // simulate, encode, or test
  {
    bool done = false;
    while (!done)    // tile loop
    {
      int nPts = 0;

      if (origHasXYZ)
      {
        fread(&nPts, 4, 1, fp);
        if (nPts == 0)    // eof
        {
          done = true;
          continue;
        }
        ptVec.resize(nPts);
        fread(&ptVec[0], sizeof(Point3D), nPts, fp);
      }

      //memset(&ptVec[0], 0, sizeof(Point3D) * nPts);    // set points to all 0

      if (origHasRGB)
      {
        int nPts2 = 0;
        fread(&nPts2, 4, 1, fp);
        if (nPts2 == 0)    // eof
        {
          done = true;
          continue;
        }
        if (origHasXYZ && nPts2 != nPts)
          return 0;

        nPts = nPts2;
        rgbVec.resize(nPts);
        fread(&rgbVec[0], sizeof(RGB_t), nPts, fp);
      }

      if (origHasIntensity)
      {
        int nPts2 = 0;
        fread(&nPts2, 4, 1, fp);
        if (nPts2 == 0)    // eof
        {
          done = true;
          continue;
        }
        if (origHasXYZ && nPts2 != nPts)
          return 0;

        nPts = nPts2;
        intensityVec.resize(nPts);
        fread(&intensityVec[0], sizeof(uint16), nPts, fp);
      }

      uint32 nBytesXYZ = 0;
      if (ptVec.size() > 0)
      {
        orderVec.resize(nPts);
        hr = lepcc_computeCompressedSizeXYZ(ctx, nPts, (const double*)(&ptVec[0]), maxXErr, maxYErr, maxZErr, &nBytesXYZ, (uint32*)(&orderVec[0]));
        if (hr)
        {
          printf("Error in main(): lepcc_computeCompressedSizeXYZ(...) failed.\n");  return 0;
        }
      }

      uint32 nBytesRGB = 0;
      if (rgbVec.size() > 0)
      {
        if (orderVec.size() > 0)    // resort the RGB values to point order
        {
          sortedRgbVec.resize(nPts);
          for (int i = 0; i < nPts; i++)
            sortedRgbVec[i] = rgbVec[orderVec[i]];

          hr = lepcc_computeCompressedSizeRGB(ctx, nPts, (const Byte*)(&sortedRgbVec[0]), &nBytesRGB);
        }
        else
          hr = lepcc_computeCompressedSizeRGB(ctx, nPts, (const Byte*)(&rgbVec[0]), &nBytesRGB);

        if (hr)
        {
          printf("Error in main(): lepcc_computeCompressedSizeRGB(...) failed.\n");  return 0;
        }
      }

      uint32 nBytesIntensity = 0;
      if (intensityVec.size() > 0)
      {
        //for (int i = 0; i < (int)intensityVec.size(); i++)
        //  intensityVec[i] = 0;

        hr = lepcc_computeCompressedSizeIntensity(ctx, nPts, &intensityVec[0], &nBytesIntensity);

        if (hr)
        {
          printf("Error in main(): lepcc_computeCompressedSizeIntensity(...) failed.\n");  return 0;
        }
      }

      int64 nBytes = nBytesXYZ + nBytesRGB + nBytesIntensity;

      // stats
      totalNumBytesCompressed += nBytes;
      totalNumPoints += nPts;
      totalNumTiles++;

      minPts = (std::min)(minPts, (double)nPts);
      maxPts = (std::max)(maxPts, (double)nPts);

      //printf("nTiles = %d, minPtsPerTile = %d, maxPtsPerTile = %d, nPtsTotal = %d\n", (int)totalNumTiles, (int)minPts, (int)maxPts, (int)totalNumPoints);

      if (mode > 0)    // encode, or test
      {
        byteVec.resize((size_t)nBytes);
        Byte* buffer = &byteVec[0];
        Byte* pByte = buffer;

        // encode
        if (ptVec.size() > 0)
          if (hr = lepcc_encodeXYZ(ctx, &pByte, nBytesXYZ))
          {
            printf("Error in main(): lepcc_encodeXYZ(...) failed.\n");  return 0;
          }

        if (rgbVec.size() > 0)
          if (hr = lepcc_encodeRGB(ctx, &pByte, nBytesRGB))
          {
            printf("Error in main(): lepcc_encodeRGB(...) failed.\n");  return 0;
          }

        if (intensityVec.size() > 0)
          if (hr = lepcc_encodeIntensity(ctx, &pByte, nBytesIntensity, &intensityVec[0], nPts))
          {
            printf("Error in main(): lepcc_encodeIntensity(...) failed.\n");  return 0;
          }

        if (mode == 1)    // encode
        {
          fwrite(buffer, 1, (size_t)nBytes, fpOut);    // append this byte blob to output stream
        }

        else if (mode == 3)    // test mode: compare decoded points to orig for this tile right away
        {
          const Byte* pByte = buffer;    // same buffer
          uint32 nPts2 = 0;

          if (!lepcc_getPointCount(ctxDec, pByte, nBytesXYZ, &nPts2) && nPts2 > 0)
          {
            decPtVec.resize(nPts2);
            if (hr = lepcc_decodeXYZ(ctxDec, &pByte, nBytesXYZ, &nPts2, (double*)(&decPtVec[0])))
            {
              printf("Error in main(): lepcc_decodeXYZ(...) failed.\n");  return 0;
            }

            // compare
            for (int i = 0; i < (int)nPts2; i++)
            {
              int k = orderVec[i];
              const Point3D* p = &ptVec[k];
              const Point3D* q = &decPtVec[i];

              double dx = abs(q->x - p->x);
              double dy = abs(q->y - p->y);
              double dz = abs(q->z - p->z);

              maxDecErrX = max(dx, maxDecErrX);
              maxDecErrY = max(dy, maxDecErrY);
              maxDecErrZ = max(dz, maxDecErrZ);
            }
          }

          if (!lepcc_getRGBCount(ctxDec, pByte, nBytesRGB, &nPts2) && nPts2 > 0)
          {
            decRgbVec.resize(nPts2);
            if (hr = lepcc_decodeRGB(ctxDec, &pByte, nBytesRGB, &nPts2, (Byte*)(&decRgbVec[0])))
            {
              printf("Error in main(): lepcc_decodeRGB(...) failed.\n");  return 0;
            }

            // compare
            bool resortedColors = !orderVec.empty();

            for (int i = 0; i < (int)nPts2; i++)
            {
              int k = resortedColors ? orderVec[i] : i;
              RGB_t rgbEnc = rgbVec[k];
              RGB_t rgbDec = decRgbVec[i];

              double dx = rgbDec.r - rgbEnc.r;
              double dy = rgbDec.g - rgbEnc.g;
              double dz = rgbDec.b - rgbEnc.b;
              double delta = sqrt(dx * dx + dy * dy + dz * dz);

              maxDecErrRGB = max(delta, maxDecErrRGB);
              totalDecErrRGB += delta;
            }
          }

          if (!lepcc_getIntensityCount(ctxDec, pByte, nBytesIntensity, &nPts2) && nPts2 > 0)
          {
            decIntensityVec.resize(nPts2);
            if (hr = lepcc_decodeIntensity(ctxDec, &pByte, nBytesIntensity, &nPts2, &decIntensityVec[0]))
            {
              printf("Error in main(): lepcc_decodeIntensity(...) failed.\n");  return 0;
            }

            // compare
            bool resortedColors = !orderVec.empty();

            for (int i = 0; i < (int)nPts2; i++)
            {
              int k = resortedColors ? orderVec[i] : i;
              uint16 intensEnc = intensityVec[k];
              uint16 intensDec = decIntensityVec[i];
              double delta = abs(intensDec - intensEnc);
              maxDecErrIntensity = max(delta, maxDecErrIntensity);
              totalDecErrIntensity += delta;
            }
          }
        }    // if (mode == 3)
      }    // if (mode > 0)
    }    // while (!done)
  }    // if (mode <= 1)

  else if (mode == 2)    // decode
  {
    bool done = false;
    while (!done)
    {
      // get info from blob header, for XYZ blob
      int nBytes = 0;
      int nPts = 0;

      if (origHasXYZ)
      {
        nBytes = ReadBlobSize(ctxDec, fp);
        
        if (nBytes == 0)
        {
          done = true;
          continue;
        }
        if (nBytes < 0)
        {
          printf("Error in main(): Read in bad num bytes for blob.\n");  return 0;
        }
      }

      if (nBytes > 0)
      {
        byteVec.resize((size_t)nBytes);
        fread(&byteVec[0], 1, (size_t)nBytes, fp);    // read it from file

        const Byte* pByte = &byteVec[0];
        uint32 nPts2 = 0;
        if ((hr = lepcc_getPointCount(ctxDec, pByte, nBytes, &nPts2)) && nPts2 > 0)
        {
          printf("Error in main(): lepcc_getPointCount(...) failed.\n");  return 0;
        }

        decPtVec.resize(nPts2);
        if ((hr = lepcc_decodeXYZ(ctxDec, &pByte, nBytes, &nPts2, (double*)(&decPtVec[0]))))
        {
          printf("Error in main(): lepcc_decodeXYZ(...) failed.\n");  return 0;
        }

        // write out the uncompressed points
        nPts = nPts2;
        fwrite(&nPts2, sizeof(int), 1, fpOut);
        fwrite(&decPtVec[0], sizeof(Point3D), nPts2, fpOut);
      }

      // get info from blob header, for RGB blob
      nBytes = 0;

      if (decodHasRGB)
      {
        nBytes = ReadBlobSize(ctxDec, fp);

        if (nBytes == 0)
        {
          done = true;
          continue;
        }
        if (nBytes < 0)
        {
          printf("Error in main(): Read in bad num bytes for blob.\n");  return 0;
        }
      }

      if (nBytes > 0)
      {
        byteVec.resize((size_t)nBytes);
        fread(&byteVec[0], 1, (size_t)nBytes, fp);    // read it from file

        const Byte* pByte = &byteVec[0];
        uint32 nPts2 = 0;
        if ((hr = lepcc_getRGBCount(ctxDec, pByte, nBytes, &nPts2)) && nPts2 > 0)
        {
          printf("Error in main(): lepcc_getRGBCount(...) failed.\n");  return 0;
        }

        decRgbVec.resize(nPts2);
        if ((hr = lepcc_decodeRGB(ctxDec, &pByte, nBytes, &nPts2, (Byte*)(&decRgbVec[0]))))
        {
          printf("Error in main(): lepcc_decodeRGB(...) failed.\n");  return 0;
        }

        // write out the uncompressed colors
        nPts = nPts2;
        fwrite(&nPts2, sizeof(int), 1, fpOut);
        fwrite(&decRgbVec[0], sizeof(RGB_t), nPts2, fpOut);
      }

      totalNumTiles++;
      totalNumPoints += nPts;
    }

    int stopSign = 0;
    fwrite(&stopSign, sizeof(int), 1, fpOut);
  }

  lepcc_deleteContext(&ctx);

  high_resolution_clock::time_point t1 = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(t1 - t0).count();

  double nBytesPerPt = (double)totalNumBytesCompressed / (double)totalNumPoints;

  std::cout << "tiles  =  " << totalNumTiles << endl;
  std::cout << "points =  " << totalNumPoints << endl;

  if (mode < 2)
  {
    std::cout << "points per tile [min, max] =  [ " << minPts << ", " << maxPts << " ]" << endl;
    std::cout << "total num bytes compressed  =  " << totalNumBytesCompressed << endl;
    std::cout << "num bytes per point         =  " << nBytesPerPt << endl;
  }
  else if (mode == 3)
  {
    std::cout << "max decoding error in [x, y, z] = [ " << maxDecErrX << ", " << maxDecErrY << ", " << maxDecErrZ << " ]" << endl;
    std::cout << "max decoding error in RGB = " << maxDecErrRGB << endl;
    std::cout << "mean decoding error in RGB = " << totalDecErrRGB / totalNumPoints << endl;
    std::cout << "max decoding error in intensity = " << maxDecErrIntensity << endl;
    std::cout << "mean decoding error in intensity = " << totalDecErrIntensity / totalNumPoints << endl;
  }

  std::cout << "total time    = " << duration << " ms" << endl;
  std::cout << "time per tile = " << ((int)((duration / totalNumTiles) * 1000 + 0.5)) * 0.001 << " ms" << endl;

  fclose(fp);

  if (fpOut)
    fclose(fpOut);

  return 0;
}

// -------------------------------------------------------------------------- ;

// only for this little test program; 
// a real app should keep track of the order compressed blobs are written to the stream, and their sizes;

int ReadBlobSize(lepcc_ContextHdl ctx, FILE* fp)
{
  Byte buffer[64];
  int nBytes = 0;
  int nInfo = lepcc_getBlobInfoSize();

  if (nInfo == fread(buffer, 1, nInfo, fp))
  {
    lepcc_blobType bt;
    uint32 blobSize = 0;
    ErrCode errCode = (ErrCode)lepcc_getBlobInfo(ctx, buffer, nInfo, &bt, &blobSize);
    nBytes = (errCode == ErrCode::Ok) ? (int)blobSize : -1;
    fseek(fp, -nInfo, SEEK_CUR);  // reset
  }

  return nBytes;
}

// -------------------------------------------------------------------------- ;
