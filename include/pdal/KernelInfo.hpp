/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <string>
#include <vector>
#include <iosfwd>

#include <pdal/pdal_internal.hpp>

namespace pdal
{

class Kernel;

class PDAL_DLL KernelInfo
{
    friend class KernelBase;
public:

    /// Constructor.
    ///
    KernelInfo(std::string const& kernelName, std::string const& kernelDescription="");

    KernelInfo& operator=(const KernelInfo& rhs);
    KernelInfo(const KernelInfo&);

    /// Destructor.
    virtual ~KernelInfo() {};

    inline std::string const& getName() const
    {
        return m_name;
    }

    inline std::string const& getDescription() const
    {
        return m_description;
    }

    inline void setInfoLink(std::string const& link)
    {
        m_link = link;
    }

    inline std::string const& getInfoLink() const
    {
        return m_link;
    }

private:
    std::string m_name;
    std::string m_description;
    std::string m_link;
};

/// Output operator for serialization
///
/// @param ostr    The output stream to write to
/// @param src     The KernelInfo to be serialized out
///
/// @return The output stream

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const KernelInfo& src);

} // namespace pdal

