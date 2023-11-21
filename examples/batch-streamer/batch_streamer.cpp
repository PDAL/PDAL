#include <pdal/io/LasReader.hpp>
#include <pdal/filters/StreamCallbackFilter.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <iostream>

// Create a producer of points read from a las file. Create a consumer. The
// consumer tells the producer when it is ready to consume points and when the
// producer should pause / resume.

double x, y, z;
int point_count = 0;
int num_total_points = 0;
std::mutex mutex;
std::condition_variable cv;

// Set this to true when a point is produced and not consumed yet
bool produced = false;

// Set this false when the producer must wait for the consumer
bool ready = false; 

void makeConsumer()
{
    while (true)
    {
        {
            // Make the consumer ready
            std::lock_guard l(mutex);
            ready = true;
        }

        cv.notify_one();
        
        {
            std::unique_lock l(mutex);
            cv.wait(l, []{ return produced; });
            
            std::cout << "Consumed point: " << x << ", " << y << ", " << z 
                      << " (count: " << point_count << ")\n";
            produced = false;   
            point_count++;
            
            // Stop when no more points to consume
            if (point_count >= num_total_points) {
                std::cout << "Consumer: no more points left.\n";
                return;
            }
        }
    }
}

bool producePoint(pdal::PointRef& p)
{
    {
        // Waiting for the consumer to be ready and to have consumed previous
        // point
        std::unique_lock<std::mutex> l(mutex);
        cv.wait(l, []{ return ready; });
        cv.wait(l, []{ return !produced; });
        
        x = p.getFieldAs<double>(pdal::Dimension::Id::X);
        y = p.getFieldAs<double>(pdal::Dimension::Id::Y);
        z = p.getFieldAs<double>(pdal::Dimension::Id::Z);
                  
        produced = true;
    }
    
    // Unlock before notifying, to avoid waking up the waiting thread only to
    // block again (see notify_one for details)        
    cv.notify_one();        

    return true;
}

void makeProducer(const std::string& filename)
{
    using namespace pdal;

    FixedPointTable t(10000);
    
    // Prepare the reader
    LasReader r;
    Options o;
    o.add("filename", filename);
    r.setOptions(o);
    QuickInfo qi = r.preview();
    num_total_points = qi.m_pointCount;
    std::cout << "Reading: " << filename << "\n";
    std::cout << "Number of points: " << num_total_points << "\n";

    // Set the precision for printing points
    std::cout.precision(10);
    
    StreamCallbackFilter f;
    f.setCallback(producePoint);
    f.setInput(r);
    f.prepare(t);
    f.execute(t);
}

int main()
{
    
    // Start the producer in a separate thread
    std::string filename = "input.las";
    std::thread t(makeProducer, filename);
    
    // Start the consumer in the main thread
    makeConsumer();
    
    t.join();
    return 0;
}
