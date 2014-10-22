/******************************************************************************
* Copyright (c) 2014, Howard Butler, hobu.inc@gmail.com
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

#include <pdal/filters/ByteSwap.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/Colorization.hpp>
#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Decimation.hpp>
#include <pdal/filters/Ferry.hpp>
#include <pdal/filters/HexBin.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/filters/Merge.hpp>
#ifdef PDAL_HAVE_PCL
#include <pdal/filters/PCLBlock.hpp>
#endif
#ifdef PDAL_HAVE_PYTHON
#include <pdal/filters/Predicate.hpp>
#include <pdal/filters/Programmable.hpp>
#endif
#include <pdal/filters/Reprojection.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/filters/Selector.hpp>
#include <pdal/filters/Sort.hpp>
#include <pdal/filters/Splitter.hpp>
#include <pdal/filters/Stats.hpp>
