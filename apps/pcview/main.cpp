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

#include <boost/timer.hpp>

#include "Engine.hpp"
#include "Controller.hpp"

#include <libpc/filters/DecimationFilter.hpp>
#include <libpc/filters/ColorFilter.hpp>
#include <libpc/drivers/las/Reader.hpp>
#include <libpc/drivers/liblas/Reader.hpp>
#include <libpc/drivers/las/Reader.hpp>
#include <libpc/Color.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/Schema.hpp>
#include <libpc/SchemaLayout.hpp>
#include <libpc/Bounds.hpp>
#include <libpc/Dimension.hpp>
#include <libpc/Iterator.hpp>

using namespace std;

static float* g_points = NULL;
static boost::uint16_t* g_colors = NULL;


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
    g_points = new float[numPoints * 3];
    g_colors = new boost::uint16_t[numPoints * 3];

    const float minx = 100.0;
    const float maxx = 200.0;
    const float miny = 50.0;
    const float maxy = 250.0;
    const float minz = 10.0;
    const float maxz = 50.0;
    const float delx = maxx-minx;
    const float dely = maxy-miny;
    const float delz = maxz-minz;

    for (int i=0; i<numPoints*3; i+=3)
    {
        float x = minx + delx*myrand_normal();
        float y = miny + dely*myrand_normal();
        float z = minz + delz*myrand_gaussian();

        g_points[i] = (float)x;
        g_points[i+1] = (float)y;
        g_points[i+2] = (float)z;

        double red,green,blue;
        libpc::Color::interpolateColor(z,minz,maxz,red,green,blue);
        
        const double vmax = (std::numeric_limits<boost::uint16_t>::max)();
        boost::uint16_t r16 = (boost::uint16_t)(red * vmax);
        boost::uint16_t g16 = (boost::uint16_t)(green * vmax);
        boost::uint16_t b16 = (boost::uint16_t)(blue * vmax);

        g_colors[i] = r16;
        g_colors[i+1] = g16;
        g_colors[i+2] = b16;
    }

    controller.setPoints(g_points, g_colors, numPoints);
    controller.setBounds(minx, miny, minz, maxx, maxy, maxz);

    return;
}


static void readFile(Controller& controller, const string& file)
{
    boost::timer timer;

    const int maxPoints = 10 * 1000;

    libpc::Stage* reader = new libpc::drivers::las::LasReader(file);
    //libpc::Stage* reader = new libpc::drivers::liblas::LiblasReader(file);
    libpc::Stage* stage = reader;
    
    printf("given %d points\n", (boost::uint32_t)stage->getNumPoints());

    libpc::Stage* decimator = NULL;
    if (reader->getNumPoints() > maxPoints)
    {
        boost::uint32_t factor = (boost::uint32_t)reader->getNumPoints() / maxPoints;
        decimator = new libpc::filters::DecimationFilter(*stage, factor);
        stage = decimator;
    }

    libpc::Stage* colorizer = NULL;
    {
        colorizer = new libpc::filters::ColorFilter(*stage);
        stage = colorizer;
    }

    boost::uint32_t numPoints = (boost::uint32_t)stage->getNumPoints();
    printf("reading of %d points started\n", numPoints);
    std::cout << "Elapsed time: " << timer.elapsed() << " seconds" << std::endl;

    const libpc::Schema& schema = stage->getSchema();
    const libpc::SchemaLayout schemaLayout(schema);
    libpc::PointBuffer buffer(schemaLayout, numPoints);

    libpc::SequentialIterator* iter = stage->createSequentialIterator();
    boost::uint32_t numRead = iter->read(buffer);
    assert(numPoints==numRead);

    printf("reading into buffer done\n", numPoints);
    std::cout << "Elapsed time: " << timer.elapsed() << " seconds" << std::endl;

    const int offsetX = schema.getDimensionIndex(libpc::Dimension::Field_X, libpc::Dimension::Int32);
    const int offsetY = schema.getDimensionIndex(libpc::Dimension::Field_Y, libpc::Dimension::Int32);
    const int offsetZ = schema.getDimensionIndex(libpc::Dimension::Field_Z, libpc::Dimension::Int32);
    const int offsetR = schema.getDimensionIndex(libpc::Dimension::Field_Red, libpc::Dimension::Uint16);
    const int offsetG = schema.getDimensionIndex(libpc::Dimension::Field_Green, libpc::Dimension::Uint16);
    const int offsetB = schema.getDimensionIndex(libpc::Dimension::Field_Blue, libpc::Dimension::Uint16);

    const libpc::Dimension& xDim = schema.getDimension(offsetX);
    const libpc::Dimension& yDim = schema.getDimension(offsetY);
    const libpc::Dimension& zDim = schema.getDimension(offsetZ);

    g_points = new float[numRead * 3];
    g_colors = new boost::uint16_t[numRead * 3];

    int cnt=0;
    for (boost::uint32_t i=0; i<numRead; i++)
    {
        const boost::int32_t xraw = buffer.getField<boost::int32_t>(i, offsetX);
        const boost::int32_t yraw = buffer.getField<boost::int32_t>(i, offsetY);
        const boost::int32_t zraw = buffer.getField<boost::int32_t>(i, offsetZ);
        
        const boost::int16_t r16 = buffer.getField<boost::int16_t>(i, offsetR);
        const boost::int16_t g16 = buffer.getField<boost::int16_t>(i, offsetG);
        const boost::int16_t b16 = buffer.getField<boost::int16_t>(i, offsetB);

        const double x = xDim.applyScaling(xraw);
        const double y = yDim.applyScaling(yraw);
        const double z = zDim.applyScaling(zraw);

        g_points[cnt] = (float)x;
        g_points[cnt+1] = (float)y;
        g_points[cnt+2] = (float)z;

        g_colors[cnt] = r16;
        g_colors[cnt+1] = g16;
        g_colors[cnt+2] = b16;

        cnt += 3;
    }

    const libpc::Bounds<double>& bounds = stage->getBounds();

    controller.setPoints(g_points, g_colors, numRead);
    controller.setBounds((float)bounds.getMinimum(0), (float)bounds.getMinimum(1), (float)bounds.getMinimum(2), 
                         (float)bounds.getMaximum(0), (float)bounds.getMaximum(1), (float)bounds.getMaximum(2));

    delete colorizer;
    delete decimator;
    delete reader;

    printf("reading of %d points done\n", numRead);
    std::cout << "Elapsed time: " << timer.elapsed() << " seconds" << std::endl;

    return;
}


int main(int argc, char** argv)
{
    Controller controller;
    //argc=1;
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

    delete[] g_points;
    delete[] g_colors;

    return 0;
}

