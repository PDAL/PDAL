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
*     * Neither the name of the Martin Isenburg or Iowa Department 
*       of Natural Resources nor the names of its contributors may be 
*       used to endorse or promote products derived from this software 
*       without specific prior written permission.
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

#include "libpc/Field.hpp"

#include <iostream>

using std::vector;
using std::string;
using std::cout;
using std::endl;


// statics
vector<Field>* Field::s_standardFields = NULL;

Field::Field() : 
  m_item(Field::InvalidItem),
  m_offset(-1),
  m_type(InvalidType)
{
  return;
}


Field::Field(DataItem item, int offset, DataType type) :
  m_item(item),
  m_offset(offset),
  m_type(type)
{
  return;
}


Field::Field(const Field& field)
{
  m_item = field.m_item;
  m_offset = field.m_offset;
  m_type = field.m_type;

  return;
}


Field& Field::operator=(const Field & other)
{
  if (this != &other)
  {
    m_item = other.m_item;
    m_offset = other.m_offset;
    m_type = other.m_type;
  }

  return *this;
}


const vector<Field>& Field::standardFields()
{
  // BUG: not threadsafe
  if (s_standardFields == NULL)
  {
    Field x(XPos, 0, F32);
    Field y(YPos, 4, F32);
    Field z(ZPos, 8, F32);

    s_standardFields = new vector<Field>();
    s_standardFields->push_back(x);
    s_standardFields->push_back(y);
    s_standardFields->push_back(z);
  }

  return *s_standardFields;
}


void Field::dump() const
{
  cout << "Field " << getName(m_item) << ": " << getName(m_type) << " at " << m_offset << endl;
}


string Field::getName(DataItem item)
{
  switch (item)
  {
  case XPos: return "XPos";
  case YPos: return "YPos";
  case ZPos: return "ZPos";
  case Time: return "Time";
  }
  return "UNKNOWN";
}


string Field::getName(DataType dt)
{
  switch (dt)
  {
  case I8: return "I8";
  case U8: return "U8";
  case I16: return "I16";
  case U16: return "U16";
  case I32: return "I32";
  case U32: return "U32";
  case I64: return "I64";
  case U64: return "U64";
  case F32: return "F32";
  case F64: return "F64";
  }
  return "UNKNOWN";
}


int Field::getSize(DataType dt)
{
  switch (dt)
  {
  case I8: return 1;
  case U8: return 1;
  case I16: return 2;
  case U16: return 2;
  case I32: return 4;
  case U32: return 4;
  case I64: return 8;
  case U64: return 8;
  case F32: return 4;
  case F64: return 8;
  }
  return 0; // BUG: notreached
}
