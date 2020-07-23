/******************************************************************************
* Copyright (c) 2020, Hobu Inc., info@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "ObjReader.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.obj",
    "Obj Reader",
    "http://pdal.io/stages/readers.obj.html",
    { "obj" }
};

CREATE_STATIC_STAGE(ObjReader, s_info)

std::string ObjReader::getName() const { return s_info.name; }
void ObjReader::addArgs(ProgramArgs& args) {}
void ObjReader::addDimensions(PointLayoutPtr layout) {}
void ObjReader::ready(PointTableRef table) {
	m_istream = Utils::openFile(m_filename, false);
}
//point_count_t ObjReader::read(PointViewPtr view, point_count_t numPts){}
void ObjReader::done(PointTableRef table){}
point_count_t ObjReader::read(PointViewPtr view, point_count_t cnt)
{
    while (true)
    {
        TRI tri;
        bool ok = readFace(tri);

	PointId pointId1, pointId2, pointId3;
        auto it = m_points.find(tri[0]);
        if (it != m_points.end())
        {
//            pointId1 = addPoint;
            m_points.insert({tri[0], pointId1});
        }
        else
            pointId1 = it->second;
        if (it != m_points.end())
        {
//            pointId2 = addPoint;
            m_points.insert({tri[1], pointId2});
        }
        else
            pointId1 = it->second;
        if (it != m_points.end())
        {
//            pointId3 = addPoint;
            m_points.insert({tri[2], pointId3});
        }
        else
            pointId1 = it->second;

        //... repeat for points 2 and 3 or stick in a lambda.

        m_mesh->add(pointId1, pointId2, pointId3);
    }
}

bool ObjReader::newVertex(double x, double y, double z)
{
	return false;
}

bool ObjReader::newTextureVertex(double x, double y, double z)
{
	return false;
}

bool ObjReader::newNormalVertex(double x, double y, double z)
{
	return false;
}

bool ObjReader::newTriangle(TRI vertices)
{
	return false;
}

bool ObjReader::readFace(TRI vertices)
{
	if(m_istream->peek() == EOF) return false;
	while(true) {
		std::string line;
		std::getline(*m_istream, line);
		auto lineType = line.substr(0, line.find(' '));
		if(lineType == "#") {
			// Coment
			// Do nothing
		}
		else if(lineType == "v") {
			// Vertex
			newVertex(0, 0, 0);
		}
		else if(lineType == "vt") {
			// Vertex texture
		}
		else if(lineType == "vn") {
			// Vertex texture
		}
		else if(lineType == "f") {
			// Face
			break;
		}
		else {
			//TODO handle this case
			throwError("Unkown line type");
		}
	}
	return true;
}

} // namespace pdal

