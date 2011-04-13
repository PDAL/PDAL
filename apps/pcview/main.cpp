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

#include <stdlib.h>
#include "Engine.hpp"
#include "Controller.hpp"

#include <libpc/filters/DecimationFilter.hpp>
#include <libpc/drivers/liblas/Reader.hpp>
#include <libpc/drivers/liblas/Iterator.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/Schema.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/Bounds.hpp>
#include <libpc/Dimension.hpp>

using namespace std;


static float myrand_gaussian() // returns [0..1]
{
    int cnt = 5;
    float x = 0.0;
    for (int i=0; i<cnt; i++)
    {
        float r = (float)rand() / (float)RAND_MAX;
        x += r;
    }
    x = x / (float)cnt;

    return x;
}


static float myrand_normal() // returns [0..1]
{
    float r = (float)rand() / (float)RAND_MAX;
    return r;
}


static void readFakeFile(Controller& controller)
{
    const int numPoints = 10000;
    static float points[numPoints * 3];

    const float minx = 100.0;
    const float maxx = 200.0;
    const float miny = 50.0;
    const float maxy = 250.0;
    const float minz = 10.0;
    const float maxz = 15.0;
    const float delx = maxx-minx;
    const float dely = maxy-miny;
    const float delz = maxz-minz;

    for (int i=0; i<numPoints*3; i+=3)
    {
        float x = minx + delx*myrand_normal();
        float y = miny + dely*myrand_normal();
        float z = minz + delz*myrand_gaussian();

        points[i] = (float)x;
        points[i+1] = (float)y;
        points[i+2] = (float)z;
    }

    controller.setPoints(points, numPoints);
    controller.setBounds(minx, miny, minz, maxx, maxy, maxz);

    return;
}


static void readFile(Controller& controller, const string& file)
{
    libpc::Stage* reader = new libpc::drivers::liblas::LiblasReader(file);
    libpc::Stage* stage = reader;
    
    libpc::Stage* decimator = NULL;
    if (reader->getNumPoints() > 50000)
    {
        boost::uint32_t factor = (boost::uint32_t)reader->getNumPoints() / 50000;
        decimator = new libpc::filters::DecimationFilter(*reader, factor);
        stage = decimator;
    }

    boost::uint32_t numPoints = (boost::uint32_t)stage->getNumPoints();
    printf("reading of %d points started\n", numPoints);

    const libpc::Schema& schema = stage->getSchema();
    const libpc::SchemaLayout schemaLayout(schema);
    libpc::PointBuffer buffer(schemaLayout, numPoints);

    libpc::SequentialIterator* iter = stage->createSequentialIterator();
    boost::uint32_t numRead = iter->read(buffer);
    assert(numPoints==numRead);

    const int offsetX = schema.getDimensionIndex(libpc::Dimension::Field_X, libpc::Dimension::Int32);
    const int offsetY = schema.getDimensionIndex(libpc::Dimension::Field_Y, libpc::Dimension::Int32);
    const int offsetZ = schema.getDimensionIndex(libpc::Dimension::Field_Z, libpc::Dimension::Int32);

    float* points = new float[numRead * 3];

    int cnt=0;
    for (boost::uint32_t i=0; i<numRead; i++)
    {
        const boost::int32_t xraw = buffer.getField<boost::int32_t>(i, offsetX);
        const boost::int32_t yraw = buffer.getField<boost::int32_t>(i, offsetY);
        const boost::int32_t zraw = buffer.getField<boost::int32_t>(i, offsetZ);

        const double x = schema.getDimension(offsetX).applyScaling(xraw);
        const double y = schema.getDimension(offsetY).applyScaling(yraw);
        const double z = schema.getDimension(offsetZ).applyScaling(zraw);

        points[cnt++] = (float)x;
        points[cnt++] = (float)y;
        points[cnt++] = (float)z;
    }

    const libpc::Bounds<double>& bounds = stage->getBounds();

    controller.setPoints(points, numRead);
    controller.setBounds((float)bounds.getMinimum(0), (float)bounds.getMinimum(1), (float)bounds.getMinimum(2), 
                         (float)bounds.getMaximum(0), (float)bounds.getMaximum(1), (float)bounds.getMaximum(2));

    delete decimator;
    delete reader;

    printf("reading of %d points done\n", numRead);

    return;
}


int main(int argc, char** argv)
{
    Controller controller;

    if (argc==1)
    {
        readFakeFile(controller);
    }
    else
    {
        readFile(controller, argv[1]);
    }

    controller.setWindowSize(500,500);
    controller.setWindowPosition(100,100);

    Engine engine(controller);

    engine.initialize(argc, argv);

    return 0;
}

