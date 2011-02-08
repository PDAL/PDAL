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

#include "libpc/FauxReader.hpp"
#include "libpc/FauxWriter.hpp"
#include "libpc/CropFilter.hpp"
#include "libpc/ColorFilter.hpp"
#include "libpc/MosaicFilter.hpp"


static void test1()
{
#if 0
  // we are faking the reader, so we need to describe it here
  // the faux reader only supports fields (X,Y,Z,T)
  const Bounds bounds(0, 200, 0, 200, -100, 100);
  const int numPoints = 30;
  FauxReader reader(bounds, numPoints);

  CropFilter cropper(reader, Bounds(0, 100, 0, 100, 0, 100));

  ColorFilter colorizer(cropper);

  FauxWriter writer(colorizer);
  
  writer.write();
#endif

  return;
}


static void test2()
{
#if 1
  const int numPoints = 10;
  const Bounds bounds1(0, 100, 0, 100, 0, 100);
  FauxReader reader1(bounds1, numPoints);

  const Bounds bounds2(100, 200, 100, 200, 0, 100);
  FauxReader reader2(bounds2, numPoints);

  MosaicFilter mosaicker(reader1, reader2);

  FauxWriter writer(mosaicker);
  
  writer.write();
#endif
  
  return;
}


int main(int /*argc*/, char* /*argv*/[])
{
  test1();

  test2();

  return 0;
}
