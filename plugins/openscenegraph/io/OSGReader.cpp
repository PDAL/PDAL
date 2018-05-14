/******************************************************************************
* Copyright (c) 2017, Jason Beverage, jasonbeverage@gmail.com
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


#include "OSGReader.hpp"
#include <pdal/util/ProgramArgs.hpp>

#include <osg/NodeVisitor>
#include <osg/TriangleFunctor>
#include <osgDB/ReadFile>
#include <osg/Version>

namespace pdal
{

struct CollectTriangles
{
    CollectTriangles()
    {
        verts = new osg::Vec3Array();
    }
#ifdef OSG_VERSION_GREATER_THAN
#if OSG_VERSION_GREATER_THAN(3,2,0)
    inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3, bool treatVertexDataAsTemporary)
#else
    inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3)
#endif
#else
    inline void operator () (const osg::Vec3& v1, const osg::Vec3& v2, const osg::Vec3& v3)
#endif
    {
        verts->push_back(v1);
        verts->push_back(v2);
        verts->push_back(v3);
    }

    osg::ref_ptr< osg::Vec3Array > verts;
};

struct CollectTrianglesVisitor : public osg::NodeVisitor
{
    CollectTrianglesVisitor() :
        osg::NodeVisitor(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN)
    {
    }

    void apply(osg::Transform& transform)
    {
        osg::Matrix matrix;
        if (!_matrixStack.empty()) matrix = _matrixStack.back();

        transform.computeLocalToWorldMatrix(matrix, this);

        pushMatrix(matrix);

        traverse(transform);

        popMatrix();
    }

    void apply(osg::Geode& geode)
    {
        for (unsigned int i = 0; i<geode.getNumDrawables(); ++i)
        {
            osg::TriangleFunctor<CollectTriangles> triangleCollector;
            geode.getDrawable(i)->accept(triangleCollector);
            for (unsigned int j = 0; j < triangleCollector.verts->size(); j++)
            {
                osg::Matrix matrix;
                if (!_matrixStack.empty())
                {
                    matrix = _matrixStack.back();
                }
                osg::Vec3d v = (*triangleCollector.verts)[j];
                _points.push_back(v * matrix);
            }
        }
    }

    inline void pushMatrix(osg::Matrix& matrix) { _matrixStack.push_back(matrix); }

    inline void popMatrix() { _matrixStack.pop_back(); }

    typedef std::vector<osg::Matrix> MatrixStack;
    std::vector< osg::Vec3d > _points;
    MatrixStack _matrixStack;
};

static PluginInfo const s_info
{
    "readers.osg",
    "osg reader.",
    ""
};

CREATE_SHARED_STAGE(OSGReader, s_info)

std::string OSGReader::getName() const { return s_info.name; }

void OSGReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
}

void OSGReader::ready(PointTableRef)
{
    osg::ref_ptr< osg::Node > node = osgDB::readNodeFile(m_filename);

    CollectTrianglesVisitor v;
    node->accept(v);
    _points = v._points;
}

point_count_t OSGReader::read(PointViewPtr view, point_count_t count)
{
    PointId nextId = view->size();
    PointId idx = m_index;
    point_count_t numRead = 0;

    for (unsigned int i = m_index; i < _points.size(); i++)
    {
        view->setField(Dimension::Id::X, nextId, _points[i].x());
        view->setField(Dimension::Id::Y, nextId, _points[i].y());
        view->setField(Dimension::Id::Z, nextId, _points[i].z());

        nextId++;
        numRead++;

        if (i >= _points.size() - 1)
        {
            break;
        }
    }

    m_index = nextId;
    return numRead;
}

void OSGReader::done(PointTableRef)
{
    _points.clear();
}

} //namespace pdal
