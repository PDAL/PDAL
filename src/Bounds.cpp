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

#include "libpc/Bounds.hpp"

#include <iostream>

using std::cout;


#define mymin(a,b) ((a)<(b)?(a):(b))
#define mymax(a,b) ((a)>=(b)?(a):(b))

void Bounds::grow(const Bounds& b)
{
  Bounds& a = *this;

  a.m_minX = mymin(a.m_minX, b.m_minX);
  a.m_maxX = mymax(a.m_maxX, b.m_maxX);

  a.m_minY = mymin(a.m_minY, b.m_minY);
  a.m_maxY = mymax(a.m_maxY, b.m_maxY);

  a.m_minZ = mymin(a.m_minZ, b.m_minZ);
  a.m_maxZ = mymax(a.m_maxZ, b.m_maxZ);

  return;
}


void Bounds::dump(void) const
{
  cout << "x(" << m_minX << "," << m_maxX << ")";
  cout << " ";
  cout << "y(" << m_minY << "," << m_maxY << ")";
  cout << " ";
  cout << "z(" << m_minZ << "," << m_maxZ << ")";
}
