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

#ifndef INCLUDED_PDALINTERNAL_HPP
#define INCLUDED_PDALINTERNAL_HPP

// This file is for all the things that basically everybody has to include.

#include <iostream>

#include <pdal/pdal_export.hpp>
#include <pdal/pdal_defines.h>
#include <pdal/pdal_types.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_macros.hpp>


// we use explicitly-sized types everywhere, so include this here
#include <boost/cstdint.hpp>
#include <boost/version.hpp>

// See http://stackoverflow.com/questions/1814548/boostsystem-category-defined-but-not-used
#ifndef BOOST_SYSTEM_NO_DEPRECATED
#define BOOST_SYSTEM_NO_DEPRECATED 1
#endif


#define PDAL_CURRENT_BOOST_MINOR_VERSION BOOST_VERSION/100%1000
#ifdef __cplusplus
#  define PDAL_C_START           extern "C" {
#  define PDAL_C_END             }
#else
#  define PDAL_C_START
#  define PDAL_C_END
#endif


#ifdef PDAL_COMPILER_MSVC
#  pragma warning(disable: 4068)  // ignore unknown pragmas (due to boost's use of GCC pragmas)
#endif

#endif
