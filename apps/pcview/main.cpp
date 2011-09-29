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
#include <boost/thread.hpp>

#include "Engine.hpp"
#include "Controller.hpp"

#include <pdal/filters/DecimationFilter.hpp>
#include <pdal/filters/ColorFilter.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/Schema.hpp>
#include <pdal/SchemaLayout.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/StageIterator.hpp>

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
    float* points = new float[numPoints * 3];
    boost::uint16_t* colors = new boost::uint16_t[numPoints * 3];

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

        points[i] = (float)x;
        points[i+1] = (float)y;
        points[i+2] = (float)z;

        double red,green,blue;
        pdal::filters::ColorFilter::interpolateColor(z,minz,maxz,red,green,blue);
        
        const double vmax = (std::numeric_limits<boost::uint16_t>::max)();
        boost::uint16_t r16 = (boost::uint16_t)(red * vmax);
        boost::uint16_t g16 = (boost::uint16_t)(green * vmax);
        boost::uint16_t b16 = (boost::uint16_t)(blue * vmax);

        colors[i] = r16;
        colors[i+1] = g16;
        colors[i+2] = b16;
    }

    controller.addPoints(points, colors, numPoints);
    controller.setBounds(minx, miny, minz, maxx, maxy, maxz);

    return;
}

    
boost::uint32_t maxPoints = 1000 * 1000;
boost::uint32_t factor = 100; //(boost::uint32_t)reader->getNumPoints() / maxPoints;
bool useColor = true;


class ThreadArgs
{
public:
    ThreadArgs(Controller& controller, pdal::Stage& stage, boost::uint32_t startPoint, boost::uint32_t numPoints)
        : m_controller(controller)
        , m_stage(stage)
        , m_startPoint(startPoint)
        , m_numPoints(numPoints)
        , m_numRead(0)
    { }

    Controller& m_controller;
    pdal::Stage& m_stage;
    boost::uint32_t m_startPoint;
    boost::uint32_t m_numPoints;
    boost::uint32_t m_numRead;

private:
    ThreadArgs(); // nope
    ThreadArgs& operator=(const ThreadArgs&); //nope
};

static boost::mutex mutex;


static void givePointsToEngine(ThreadArgs* threadArgs)
{
    Controller& controller = threadArgs->m_controller;
    pdal::Stage& stage = threadArgs->m_stage;
    boost::uint32_t startPoint = threadArgs->m_startPoint;
    boost::uint32_t numPoints = threadArgs->m_numPoints;

    const pdal::Schema& schema = stage.getSchema();

    pdal::StageSequentialIterator* iter = stage.createSequentialIterator();
    pdal::PointBuffer buffer(schema, numPoints);
    iter->skip(startPoint);
    const boost::uint32_t numRead = iter->read(buffer);

    float* points = new float[numRead * 3];
    boost::uint16_t *colors = NULL;
    if (useColor)
    {
        colors = new boost::uint16_t[numRead * 3];
    }

    const int offsetX = schema.getDimensionIndex(pdal::DimensionId::X_i32);
    const int offsetY = schema.getDimensionIndex(pdal::DimensionId::Y_i32);
    const int offsetZ = schema.getDimensionIndex(pdal::DimensionId::Z_i32);
    const int offsetR = schema.getDimensionIndex(pdal::DimensionId::Red_u16);
    const int offsetG = schema.getDimensionIndex(pdal::DimensionId::Green_u16);
    const int offsetB = schema.getDimensionIndex(pdal::DimensionId::Blue_u16);

    const pdal::Dimension& xDim = schema.getDimension(pdal::DimensionId::X_i32);
    const pdal::Dimension& yDim = schema.getDimension(pdal::DimensionId::Y_i32);
    const pdal::Dimension& zDim = schema.getDimension(pdal::DimensionId::Z_i32);

    int cnt=0;
    for (boost::uint32_t i=0; i<numRead; i++)
    {
        const boost::int32_t xraw = buffer.getField<boost::int32_t>(i, offsetX);
        const boost::int32_t yraw = buffer.getField<boost::int32_t>(i, offsetY);
        const boost::int32_t zraw = buffer.getField<boost::int32_t>(i, offsetZ);
        
        const double x = xDim.applyScaling(xraw);
        const double y = yDim.applyScaling(yraw);
        const double z = zDim.applyScaling(zraw);

        points[cnt] = (float)x;
        points[cnt+1] = (float)y;
        points[cnt+2] = (float)z;

        if (useColor)
        {
            const boost::int16_t r16 = buffer.getField<boost::int16_t>(i, offsetR);
            const boost::int16_t g16 = buffer.getField<boost::int16_t>(i, offsetG);
            const boost::int16_t b16 = buffer.getField<boost::int16_t>(i, offsetB);

            colors[cnt] = r16;
            colors[cnt+1] = g16;
            colors[cnt+2] = b16;
        }

        cnt += 3;
    }

    mutex.lock();
    controller.addPoints(points, colors, numRead);
    mutex.unlock();

    delete iter;

    threadArgs->m_numRead = numRead;

    return;
}


static void readFileSimple(Controller& controller, const string& file)
{
    boost::timer timer;

    pdal::Stage* reader = new pdal::drivers::las::Reader(file);
    
    pdal::Stage* decimator = new pdal::filters::DecimationFilter(*reader, factor);

    pdal::Stage* colorizer = new pdal::filters::ColorFilter(*decimator);

    colorizer->initialize();

    const boost::uint32_t numPoints = (boost::uint32_t)colorizer->getNumPoints();

#if 1
    ThreadArgs t1arg(controller, *colorizer, 0, numPoints);
    givePointsToEngine(&t1arg);
    const boost::uint32_t numRead1 = t1arg.m_numRead;
    cout << "moved " << numRead1 << " points into buffer1 done\n";
#else
    ThreadArgs t1arg(controller, *colorizer, 0, numPoints/2);
    ThreadArgs t2arg(controller, *colorizer, numPoints/2, numPoints/2);
    boost::thread t1(givePointsToEngine, &t1arg);
    boost::thread t2(givePointsToEngine, &t2arg);

    t1.join();
    t2.join();

    const boost::uint32_t numRead1 = t1arg.m_numRead;
    const boost::uint32_t numRead2 = t2arg.m_numRead;

    cout << "moved " << numRead1 << " points into buffer1 done\n";
    cout << "moved " << numRead2 << " points into buffer2 done\n";
#endif

    cout << "done reading points\n";
    cout << "  elapsed time: " << timer.elapsed() << " seconds" << std::endl;
    
    const pdal::Bounds<double>& bounds = colorizer->getBounds();
    controller.setBounds((float)bounds.getMinimum(0), (float)bounds.getMinimum(1), (float)bounds.getMinimum(2), 
                         (float)bounds.getMaximum(0), (float)bounds.getMaximum(1), (float)bounds.getMaximum(2));

    delete colorizer;
    delete decimator;
    delete reader;

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
        //readFile(controller, argv[1]);
        readFileSimple(controller, argv[1]);
    }

    controller.setWindowSize(500,500);
    controller.setWindowPosition(100,100);

    Engine engine(controller);

    engine.initialize(argc, argv);

    //delete[] g_points;
    //delete[] g_colors;

    return 0;
}

