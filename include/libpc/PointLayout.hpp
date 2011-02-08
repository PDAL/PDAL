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
#include <string>

#include "libpc/Field.hpp"


class PointLayout
{
public:
  PointLayout();
  PointLayout(const PointLayout&);
  PointLayout& operator=(const PointLayout& other);

  bool operator==(const PointLayout& other) const;

  // adds a field to the end of the layout, and returns the index of the field added
  int addField(const Field& field);

  // returns a given field
  const Field& getField(int fieldIndex) const { return m_fields[fieldIndex]; }
  Field& getField(int fieldIndex) { return m_fields[fieldIndex]; }

  // total num bytes for all fields in the point
  int getSizeInBytes() const;

  // number of fields in the point
  int getNumFields() const;

  void dump(std::string indent="") const;

  // returns -1 if not found
  int findFieldIndex(Field::DataItem item) const;
  bool hasField(Field::DataItem item) const;

  // some well-known field types are always available
  // (is this worth it?)
  int getFieldIndex_X() const { return m_fieldIndex_X; }
  int getFieldIndex_Y() const { return m_fieldIndex_Y; }
  int getFieldIndex_Z() const { return m_fieldIndex_Z; }

  void markAllFieldsInactive();

private:
  std::vector<Field> m_fields; // each of the fields

  int m_numBytes; // num bytes required to store all fields

  // local cache of handy values
  int m_fieldIndex_X;
  int m_fieldIndex_Y;
  int m_fieldIndex_Z;
};

#endif
