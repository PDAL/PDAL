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

#include <iostream>

#include "libpc/FauxWriter.hpp"

using std::string;
using std::cout;
using std::endl;

using namespace libpc;

FauxWriter::FauxWriter(Stage& prevStage) :
    Writer(prevStage)
{
    return;
}


void FauxWriter::writeBegin()
{
    cout << "FauxWriter::writeBegin()" << endl;
    cout << endl;

    m_numPointsWritten = 0;

    getHeader().dump();

    return;
}


void FauxWriter::writeEnd()
{
    cout << "FauxWriter::writeEnd()" << endl;
    cout << "  wrote " << m_numPointsWritten << " points" << endl;
    cout << endl;

    return;
}


void FauxWriter::writeBuffer(const PointData& pointData)
{
    const int numPoints = pointData.getNumPoints();

    int numValidPoints = 0;
    for (int pointIndex=0; pointIndex<numPoints; pointIndex++)
    {
        if (pointData.isValid(pointIndex))
        {
            ++numValidPoints;
        }
    }

    cout << "FauxWriter::writeBuffer()" << endl;
    cout << "  writing " << numValidPoints << " of " << numPoints << " points" << endl;
    cout << endl;

    pointData.dump(" ");

    cout << endl;

    return;
}
