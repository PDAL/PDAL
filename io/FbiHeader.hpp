/******************************************************************************
 * Copyright (c) 2021, Antoine Lavenant, antoine.lavenant@ign.fr
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

#pragma once

#include <memory>
#include <vector>

#include <pdal/Log.hpp>

namespace pdal
{

namespace fbi
{

//typedef from FBI documentation
typedef uint32_t UINT;
typedef uint64_t UINT64;
typedef unsigned char BYTE ;

// Normal vector & dimensionality
typedef struct {
  unsigned Dim     :  2 ; // Dimension 0=unknown, 1=line, 2=plane, 3=random
  unsigned HorzAng : 15 ; // Horizontal angle
  unsigned VertAng : 15 ; // Vertical angle
} NrmVec ;

static double hc_pi = 3.1415926535897932384626433;
static double hc_2pi = 2.0 * hc_pi ;
static double hc_piover2 = 0.5 * hc_pi ;

//struct from FBI documentation
struct FbiHdr {
    
    char Signature[8] = "FASTBIN";
    UINT Version ; // Version number = 1
    UINT HdrSize ; // sizeof(FbiHdr)
    UINT TimeType ; // 0=GPS seconds-of-week, 1=GPS standard time
    UINT Order ; // Point order: 0=not known, 1=time order, 2=display optim.
    UINT Reserved1; // Set to zero
    UINT VlrCnt ; // Number of variable length records (0 - n)
    UINT VlrSize ; // Combined length of variable length records
    UINT RecSize ; // Size of additional point records
    
    UINT64 FastCnt ; // Number of fast points (=stream points)
    UINT64 RecCnt ; // Number of additional point records
    UINT UnitsXyz ; // Xyz scale factor (=integer steps per master unit)
    UINT UnitsDistance ; // Distance scale factor (=integer steps per master unit)
    double OrgX ; // Origin X
    double OrgY ; // Origin Y
    double OrgZ ; // Origin Z
    double MinX ; // X minimum of points
    double MaxX ; // X maximum of points
    double MinY ; // Y minimum of points
    double MaxY ; // Y maximum of points
    double MinZ ; // Z minimum of points
    double MaxZ ; // Z maximum of points
    char System[32] = "" ; // System identifier
    char Software[32] = "PDAL"; // Generating software
    char Reserved2[64] = ""; // Reserved for future use
    
    UINT BitsX ; // Number of bits for X (32)
    UINT BitsY ; // Number of bits for Y (32)
    UINT BitsZ ; // Number of bits for Z (32)
    UINT BitsTime ; // Number of bits for time (0 or 64)
    UINT BitsDistance ; // Number of bits for distance (0 or 32)
    UINT BitsGroup ; // Number of bits for group (0 or 32)
    UINT BitsNormal ; // Number of bits for normal vectors (0 or 32)
    UINT BitsColor ; // Number of bits for color (0, 48 or 64)
    UINT BitsIntensity ; // Number of bits for intensity (0 or 16)
    UINT BitsLine ; // Number of bits for line (0 or 16)
    UINT BitsEchoLen ; // Number of bits for echo length (0 or 16)
    UINT BitsAmplitude ; // Number of bits for amplitude (0 or 16)
    UINT BitsScanner ; // Number of bits for scanner (0 or 8)
    UINT BitsEcho ; // Number of bits for echo (0 or 8)
    UINT BitsAngle ; // Number of bits for angle (0 or 8)
    UINT BitsEchoNorm ; // Number of bits for echo normality (0 or 8)
    UINT BitsClass ; // Number of bits for class (0 or 8)
    UINT BitsEchoPos ; // Number of bits for echo position (0 or 16)
    UINT BitsImage ; // Number of bits for image indexes (0, 16 or 32)
    UINT BitsReflect ; // Number of bits for reflectance (0 or 16)
    UINT BitsDeviation ; // Number of bits for deviation (0 or 16)
    UINT BitsReliab ; // Number of bits for reliability (0 or 8)
    
    UINT64 Reserved5 ; // Set to zero
    UINT64 PosVlr ; // File position of VLR records
    UINT64 PosXyz ; // File position of xyz bit stream
    UINT64 PosTime ; // File position of time bit stream
    UINT64 PosDistance ; // File position of distance bit stream
    UINT64 PosGroup ; // File position of group bit stream
    UINT64 PosNormal ; // File position of normal vectors bit stream
    UINT64 PosColor ; // File position of color bit stream
    UINT64 PosIntensity ; // File position of intensity bit stream
    UINT64 PosLine ; // File position of line bit stream
    UINT64 PosEchoLen ; // File position of echo length bit stream
    UINT64 PosAmplitude ; // File position of amplitude bit stream
    UINT64 PosScanner ; // File position of scanner bit stream
    UINT64 PosEcho ; // File position of echo bit stream
    UINT64 PosAngle ; // File position of angle bit stream
    UINT64 PosEchoNorm ; // File position of echo normality bit stream
    UINT64 PosClass ; // File position of class bit stream
    UINT64 PosRecord ; // File position of additional point records
    UINT64 PosEchoPos ; // File position of echo position bit stream
    UINT64 PosImage ; // File position of image indexes
    UINT64 PosReflect ; // File position of reflectance
    UINT64 PosDeviation ; // File position of deviation
    UINT64 PosReliab ; // File position of reliability
    UINT64 PosImgNbr ; // File position of image number list
    UINT ImgNbrCnt ; // Number of 64 bit image numbers at PosImgNbr
    char Reserved6[1260]{}; // Fill with zeroes
    
    void dump(const LogPtr& log);
};

} // namespace fbi
} // namespace pdal
