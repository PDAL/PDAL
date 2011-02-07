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

#include "libpc/PointLayout.hpp"

#include <iostream>
using std::cout;
using std::endl;


PointLayout::PointLayout() :
  m_offsetX(-1),
  m_offsetY(-1),
  m_offsetZ(-1)
{
  return;
}


PointLayout::PointLayout(const PointLayout& other)
{
  m_fields = other.m_fields;
  m_offsetX = other.m_offsetX;
  m_offsetY = other.m_offsetY;
  m_offsetZ = other.m_offsetZ;

  return;
}


PointLayout& PointLayout::operator=(const PointLayout& other)
{
  if (this != &other)
  {
    m_fields = other.m_fields;
    m_offsetX = other.m_offsetX;
    m_offsetY = other.m_offsetY;
    m_offsetZ = other.m_offsetZ;
  }

  return *this;
}


void PointLayout::addField(const Field& field)
{
  m_fields.push_back(field);

  if (field.item() == Field::XPos)
  {
    m_offsetX = field.offset();
  }
  else  if (field.item() == Field::YPos)
  {
    m_offsetY = field.offset();
  }
  else if (field.item() == Field::ZPos)
  {
    m_offsetZ = field.offset();
  }

  return;
}


void PointLayout::addFields(const std::vector<Field>& fields)
{
  for (size_t i=0; i<fields.size(); i++)
  {
    addField(fields[i]);
  }
}


void PointLayout::dump() const
{
  cout << "PointLayout:" << endl;

  for (size_t i=0; i<m_fields.size(); i++)
  {
    m_fields[i].dump();
  }
}


int PointLayout::getSizeInBytes() const
{
  int size = 0;

  for (size_t i=0; i<m_fields.size(); i++)
  {
    const Field& f = m_fields[i];

    int pos = f.offset() + Field::getSize(f.type());
    if (pos > size) size = pos;
  }

  return size;
}


int PointLayout::getNumFields() const
{
  return m_fields.size();
}


bool PointLayout::findField(Field::DataItem item, Field& ret) const
{
  for (size_t i=0; i<m_fields.size(); i++)
  {
    const Field& f = m_fields[i];
    if (f.item() == item) 
    {
      ret = f;
      return true;
    }
  }

  return false;
}


int PointLayout::findFieldOffset(Field::DataItem item) const
{
  Field ret;
  bool ok = findField(item, ret);
  if (ok) return ret.offset();

  return -1;
}
