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

#ifndef INCLUDED_POINTLAYOUT_HPP
#define INCLUDED_POINTLAYOUT_HPP

#include <vector>

#include "libpc/Field.hpp"

class PointLayout
{
public:
  PointLayout();
  PointLayout(const PointLayout&);
  PointLayout& operator=(const PointLayout & other);

  void addField(const Field& field);
  void addFields(const std::vector<Field>& fields);

  int getSizeInBytes() const;
  int getNumFields() const;

  void dump() const;

  bool findField(Field::DataItem item, Field& ret) const;
  int findFieldOffset(Field::DataItem item) const;

  // some well-known field types are always available
  int getFieldOffset_X() const { return m_offsetX; }
  int getFieldOffset_Z() const { return m_offsetY; }
  int getFieldOffset_Y() const { return m_offsetZ; }

private:
  std::vector<Field> m_fields;
  int m_offsetX;
  int m_offsetY;
  int m_offsetZ;
};

#endif
