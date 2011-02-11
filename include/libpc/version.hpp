/******************************************************************************
 * $Id$
 *
 * Project:  libLAS - http://liblas.org - A BSD library for LAS format data.
 * Purpose:  Version information
 * Author:   Mateusz Loskot, mateusz@loskot.net
 *
 ******************************************************************************
 * Copyright (c) 2010, Mateusz Loskot
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

#ifndef LIBPC_VERSION_HPP_INCLUDED
#define LIBPC_VERSION_HPP_INCLUDED

#include <string>
#include "libpc/export.hpp"

// Caution, this is the only libPC header that is guarenteed to change with
// every libPC release; including this header will cause a recompile every
// time a new libPC version is released.


// When changing the version numbers of libPC, you should ONLY modify the following
// three macros:
#define LIBPC_VERSION_MAJOR 0
#define LIBPC_VERSION_MINOR 1
#define LIBPC_VERSION_PATCH 0


// we use a nine digit code of the form "xxxyyyzzz" for the version,
//   where xxx=major, yyy=minor, and zzz=patch

#define LIBPC_VERSION ((LIBPC_VERSION_MAJOR * 1000 * 1000) + (LIBPC_VERSION_MINOR * 1000) + LIBPC_VERSION_PATCH)


// LIBLAS_LIB_VERSION must be defined to be the same as LIBLAS_VERSION
// but as a *string* in the form "x_y_z" where x is the major version
// number, y is the minor version number, and z is the patch level.

#define LIBPC_XSTRINGIFY(str) #str
#define LIBPC_STRINGIFY(str) LIBPC_XSTRINGIFY(str)

#define LIBPC_VERSION_STRING LIBPC_STRINGIFY(LIBPC_VERSION_MAJOR) ## "_" LIBPC_STRINGIFY(LIBPC_VERSION_MINOR) ## "_" ##  LIBPC_STRINGIFY(LIBPC_VERSION_PATCH)


namespace libpc
{

bool LIBPC_DLL IsGDALEnabled(void);
bool LIBPC_DLL IsLibGeoTIFFEnabled(void);
bool LIBPC_DLL IsLasZipEnabled(void);
std::string LIBPC_DLL GetFullVersion(void);
std::string LIBPC_DLL GetVersion(void);

} // namespace liblas

#endif // LIBPC_VERSION_HPP_INCLUDED
