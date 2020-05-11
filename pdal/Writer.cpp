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

#include <pdal/Writer.hpp>
#include <pdal/Stage.hpp>

#include <pdal/util/ProgramArgs.hpp>

#pragma warning(disable: 4127)  // conditional expression is constant

namespace pdal
{

Writer::Writer()
{}

Writer::~Writer()
{}

std::string::size_type Writer::handleFilenameTemplate(
    const std::string& filename)
{
    std::string::size_type suffixPos = filename.find_last_of('.');
    std::string::size_type hashPos = filename.find_first_of('#');
    if (hashPos == std::string::npos)
        return hashPos;

    if (hashPos > suffixPos)
    {
        std::ostringstream oss;
        oss << "Filename template placeholder ('#') is not "
            "allowed in filename suffix.";
        throw pdal_error(oss.str());
    }
    if (filename.find_first_of('#', hashPos + 1) != std::string::npos)
    {
        std::ostringstream oss;
        oss << "Filename specification can only contain "
            "a single '#' template placeholder.";
        throw pdal_error(oss.str());
    }
    return hashPos;
}

} // namespace pdal
