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

#ifndef INCLUDED_DRIVERS_LAS_GEOTIFFSUPPORT_HPP
#define INCLUDED_DRIVERS_LAS_GEOTIFFSUPPORT_HPP

#include <pdal/pdal.hpp>

#ifdef LIBPC_HAVE_LIBGEOTIFF
#include <geo_simpletags.h>
#include <cpl_conv.h>
#endif

#include <string>
#include <stdexcept>

namespace pdal { namespace drivers { namespace las {
    

class LIBPC_DLL GeotiffSupport
{
public:
    enum GeotiffKeyType
    {
        Geotiff_KeyType_SHORT=1,
        Geotiff_KeyType_DOUBLE=2,
        Geotiff_KeyType_ASCII=3
    };

public:
    GeotiffSupport();
    ~GeotiffSupport();
    
    void resetTags();
    int setKey(int tag, int count, GeotiffKeyType geotiff_key_type, void *data);
    int getKey(int tag, int *count, int *st_type, void **data_ptr) const;
    void setTags();

    std::string getWkt(bool horizOnly, bool pretty) const;
    void setWkt(const std::string&);

    std::string getText() const;

private:
    void rebuildGTIF();

    GTIF* m_gtiff;
    ST_TIFF* m_tiff;
};


} } } // namespace

#endif
