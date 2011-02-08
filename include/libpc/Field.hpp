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

#ifndef INCLUDED_FIELD_HPP
#define INCLUDED_FIELD_HPP

#include <vector>
#include <string>

class Field
{
public:
  enum DataItem
  {
    InvalidItem,
    XPos,
    YPos,
    ZPos,
    Time,
    Zred,
    Zgreen,
    Zblue
  };
  static std::string getName(DataItem);

  enum DataType
  {
    InvalidType,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64
  };
  static std::string getName(DataType);
  static int getSize(DataType);

public:
  Field();
  Field(DataItem item, DataType type);
  Field(const Field&);

  Field& operator=(const Field&);
  bool operator==(const Field& other) const;

  DataItem getItem() const { return m_item; }
  DataType getType() const { return m_type; }

  int getOffset() const { return m_offset; }
  void setOffset(int offset) { m_offset = offset; }

  int getIndex() const { return m_index; }
  void setIndex(int index) { m_index = index; }

  int getNumBytes() const;

  static const std::vector<Field>& standardFields();

  void dump() const;

private:
  DataItem m_item;
  DataType m_type;

  int m_offset;// byte offset within a point buffer
  int m_index; // 0..n-1, where n is number of fields in layout

  static std::vector<Field>* s_standardFields;
};

#endif
