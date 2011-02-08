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

#ifndef INCLUDED_BOUNDS_HPP
#define INCLUDED_BOUNDS_HPP

#include <limits>

class Bounds
{
public:
  Bounds()
    : m_minX(-std::numeric_limits<double>::infinity()),
    m_maxX(std::numeric_limits<double>::infinity()),
    m_minY(-std::numeric_limits<double>::infinity()),
    m_maxY(std::numeric_limits<double>::infinity()),
    m_minZ(-std::numeric_limits<double>::infinity()),
    m_maxZ(std::numeric_limits<double>::infinity())
  {
  }

  Bounds(double minX, double maxX, double minY, double maxY, double minZ, double maxZ)
    : m_minX(minX),
    m_maxX(maxX),
    m_minY(minY),
    m_maxY(maxY),
    m_minZ(minZ),
    m_maxZ(maxZ)
  {
  }

  Bounds(const Bounds& other)
    : m_minX(other.m_minX),
    m_maxX(other.m_maxX),
    m_minY(other.m_minY),
    m_maxY(other.m_maxY),
    m_minZ(other.m_minZ),
    m_maxZ(other.m_maxZ)
  {
  }

  Bounds& operator=(const Bounds & other)
  {
    if (this != &other)
    {
      m_minX = other.m_minX;
      m_maxX = other.m_maxX;
      m_minY = other.m_minY;
      m_maxY = other.m_maxY;
      m_minZ = other.m_minZ;
      m_maxZ = other.m_maxZ;
    }
    return *this;
  }

  bool contains(double x, double y, double z) const
  {
    return (x >= m_minX && x <= m_maxX && y >= m_minY && y <= m_maxY && z >= m_minZ && z <= m_maxZ);
  }

  // enlarge the bbox of 'this' to include the points of 'other'
  void grow(const Bounds& other);

  void dump() const;

  double m_minX;
  double m_maxX;
  double m_minY;
  double m_maxY;
  double m_minZ;
  double m_maxZ;
};

#endif
