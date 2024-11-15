/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (hobu.inc@gmail.com)
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

#include <map>

#include <pdal/Kernel.hpp>
#include <filters/SplitterFilter.hpp>

namespace pdal
{

class PDAL_EXPORT TileKernel : public Kernel
{
    using Coord = std::pair<int, int>;
    using Readers = std::map<std::string, Streamable *>;

public:
    TileKernel();
    std::string getName() const;
    int execute();

private:
    void addSwitches(ProgramArgs& args);
    void validateSwitches(ProgramArgs& args);
    Streamable *prepareReader(const std::string& filename);
    void process(const Readers& readers);
    void checkReaders(const Readers& readers);
    void adder(PointRef& point, int xpos, int ypos);

    std::string m_inputFile;
    std::string m_outputFile;
    double m_length;
    double m_xOrigin;
    double m_yOrigin;
    double m_buffer;
    std::map<Coord, Streamable *> m_writers;
    FixedPointTable m_table;
    SplitterFilter m_splitter;
    Streamable *m_repro;
    SpatialReference m_outSrs;
    std::string::size_type m_hashPos;
};

} // namespace pdal
