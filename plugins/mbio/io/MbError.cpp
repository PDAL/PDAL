/******************************************************************************
* Copyright (c) 2017, Howard Butler (hobu@hob.co)
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

#include <map>

#include "MbFormat.hpp"

typedef std::pair<int, std::string> MbError;

namespace pdal
{

namespace MbError
{

namespace
{

std::map<int, std::string> errors =
{
    { 0, "No error." },
    { 1, "Memory allocation failure." },
    { 2, "Can't open file." },
    { 3, "Bad format." },
    { 4, "End of file detected." },
    { 5, "Write failure." },
    { 6, "No beams in bounds." },
    { 7, "No beams in time window." },
    { 8, "Bad descriptor." },
    { 9, "Bad usage." },
    { 10, "No pings binned." },
    { 11, "Bad record type (kind)." },
    { 12, "Bad parameter." },
    { 13, "Bad buffer ID." },
    { 14, "Bad system." },
    { 15, "Bad data." },
    { 16, "Missing data." },
    { -1, "Time gap." },
    { -2, "Position out of bounds." },
    { -3, "Time out of bounds." },
    { -4, "Speed too small." },
    { -5, "Comment." },
    { -6, "Sub-bottom." },
    { -7, "Water column." },
    { -8, "Other." },
    { -9, "Unintelligible." },
    { -10, "Ignore." },
    { -11, "No data requested." },
    { -12, "Buffer full." },
    { -13, "No data loaded." },
    { -14, "Buffer empty." },
    { -15, "No data dumped." },
    { -16, "No more data." },
    { -17, "Data not inserted." },
    { -18, "Bad projection." },
    { -19, "Missing projections." },
    { -20, "Missing Navattitude." },
    { -21, "Not enough data." },
    { -22, "File not found." },
    { -23, "File locked." },
    { -24, "Initialization failure." },
};

} // unnamed namespace

std::string text(int errorCode)
{
    auto ei = errors.find(errorCode);
    if (ei != errors.end())
        return ei->second;
    return "";
}

} // namespace MbError

} // namespace pdal
