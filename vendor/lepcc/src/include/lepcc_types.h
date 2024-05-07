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

Contributors:  Thomas Maurer, Ronald Poirrier
*/

#pragma once

//#define TryHuffmanOnColor

namespace lepcc
{

  typedef unsigned char Byte;
  typedef unsigned short uint16;
  typedef unsigned int uint32;
  typedef long long int64;
  typedef unsigned long long uint64;

  struct RGB_t
  {
    Byte r, g, b;

    RGB_t() : r{0}, g{0}, b{0}
    {}
    RGB_t(Byte r0, Byte g0, Byte b0) : r(r0), g(g0), b(b0)
    {}

    bool operator==(const RGB_t& v) const
    { return r == v.r && g == v.g && b == v.b; }
  };

  struct Point3D
  {
    double x, y, z;

    Point3D() : x{0}, y{0}, z{0}
    {}

    Point3D(double a, double b, double c) : x(a), y(b), z(c)
    {}

    Point3D operator-(const Point3D& b) const
    { return Point3D(x - b.x, y - b.y, z - b.z); }
  };

  struct Extent3D
  {
    Point3D lower, upper;
  };

  enum class ErrCode : int
  {
    Ok,
    Failed,
    WrongParam,
    WrongVersion,
    WrongCheckSum,
    NotLepcc,
    NotClusterRGB,
    NotIntensity,
    NotFlagBytes,
    BufferTooSmall,
    OutArrayTooSmall,
    QuantizeVirtualRasterTooBig,
    QuantizeIndexOutOfRange
  };

  enum class BlobType : int
  {
    bt_XYZ = 0,
    bt_RGB,
    bt_Intensity,
    bt_FlagBytes
  };

}    // namespace

