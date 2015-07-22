/******************************************************************************
* Copyright (c) 2014, Michael P. Gerlek (mpg@flaxen.com)
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <pdal/pdal_internal.hpp>

#include <vector>

#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic push
#  pragma GCC diagnostic ignored "-Wredundant-decls"
#  pragma GCC diagnostic ignored "-Wextra"
#  pragma GCC diagnostic ignored "-Wcast-qual"
   // The following pragma doesn't actually work:
   //   https://gcc.gnu.org/bugzilla/show_bug.cgi?id=61653
   //#  pragma GCC diagnostic ignored "-Wliteral-suffix"
#endif
#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic push
#  pragma clang diagnostic ignored "-Wunused-private-field"
#endif

#define IMPORT_NITRO_API
#include <nitro/c++/import/nitf.hpp>

#ifdef PDAL_COMPILER_CLANG
#  pragma clang diagnostic pop
#endif
#ifdef PDAL_COMPILER_GCC
#  pragma GCC diagnostic pop
#endif

namespace pdal
{
    class MetadataNode;
}


namespace pdal
{

//
// helper class for processing all the metadata fields
//
class PDAL_DLL MetadataReader
{
public:
    MetadataReader(::nitf::Record&, MetadataNode&, bool showEmptyFields=true);
    ~MetadataReader();

    void read();

private:
    ::nitf::Record m_record;
    MetadataNode& m_node;
    const bool m_showEmptyFields;

    void writeField(const std::string& parentkey,
                    const std::string& key,
                    ::nitf::Field field);
    void writeInt(const std::string& parentkey,
                  const std::string& key,
                  int thevalue);
    void writeString(const std::string& parentkey,
                     const std::string& key,
                     const std::string& thevalue);

    void doFileHeader(const std::string& parentkey,
                      ::nitf::FileHeader&);

    void doSecurity(const std::string& parentkey,
                    const std::string& prefix,
                    ::nitf::FileSecurity&);

    void doBands(const std::string& key,
                 ::nitf::ImageSubheader&);
    void doBand(const std::string& key,
                 ::nitf::BandInfo&);

    void doImageSubheader(const std::string& key,
                          ::nitf::ImageSubheader&);

    void doDESubheader(const std::string& key,
                      ::nitf::DESubheader&);

    void doExtensions(const std::string& key,
                      ::nitf::Extensions&);

    void doComments(const std::string& key,
                    ::nitf::List&);

    void doTRE(const std::string& key,
               ::nitf::TRE&);

    MetadataReader(const MetadataReader&); // nope
    MetadataReader& operator=(const MetadataReader&); // nope
};


} // namespaces
