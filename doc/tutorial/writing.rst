.. _writing:

=====================
Writing with PDAL
=====================

This tutorial will describe a complete example of using PDAL C++ objects to write
a LAS file. The example will show fetching data from your own data source rather than
interacting with a :ref:`reader stage <stage_index>`.

.. note::

     If you implement your own :ref:`reader <stage_index>` that conforms to
     PDAL's :cpp:class:`pdal::Stage`, you can implement a simple read-filter-write
     pipeline using :ref:`pipeline` and not have to code anything explicit
     yourself.

Compilation
-------------------------------------------------------------------------------

To build this example create a file called `pdal-tutorial.cpp` in the root of your PDAL tree and
issue a compilation command something like the following:


::

    g++ -g -std=c++11 -o pdal-tutorial -I./include -L./lib -lpdalcpp pdal-tutorial.cpp

.. note::

    Refer to :ref:`building` for information on how to build PDAL.


Includes
-------------------------------------------------------------------------------

As of PDAL 1.0.0, there's a couple of include files available for use.  The
main one, `<pdal/pdal.hpp>`, brings in most of the utility and basic classes
used throughout the PDAL system. Additionally, the following includes are
available subject to having the required include directories pathed:

.. code-block:: cpp

    #include <pdal/pdal.hpp>
    #include <pdal/PointBuffer.hpp>
    #include <BufferReader.hpp>
    #include <LasWriter.hpp>

    #include <vector>

    struct Point
    {
        double x;
        double y;
        double z;
    };

    std::vector<Point> getMyData()
    {

        std::vector<Point> output;
        Point p;

        for (int i = 0; i < 1000; ++i)
        {
            p.x = -93.0 + i*0.001;
            p.y = 42.0 + i*0.001 ;
            p.z = 106.0 + i;
            output.push_back(p);
        }
        return output;
    }


    void fillBuffer(pdal::PointBufferPtr buffer, std::vector<Point> const& data)
    {

        for (int i = 0; i < data.size(); ++i)
        {
            Point const& pt = data[i];
            buffer->setField<double>(pdal::Dimension::Id::X, i, pt.x);
            buffer->setField<double>(pdal::Dimension::Id::Y, i, pt.y);
            buffer->setField<double>(pdal::Dimension::Id::Z, i, pt.z);
        }
    }

    int main( int argc, const char* argv[] )
    {

        pdal::Options options;

        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 7, "");
        // options.add(debug);
        // options.add(verbose);

        pdal::Option filename("filename", "myfile.las");
        options.add(filename);
        pdal::PointContextRef ctx;

        ctx.registerDim(pdal::Dimension::Id::X);
        ctx.registerDim(pdal::Dimension::Id::Y);
        ctx.registerDim(pdal::Dimension::Id::Z);

        {
            pdal::PointBufferPtr buffer = pdal::PointBufferPtr(new pdal::PointBuffer(ctx));

            std::vector<Point> data = getMyData();

            fillBuffer(buffer, data);

            pdal::BufferReader reader(options);
            reader.addBuffer(buffer);
            pdal::LasWriter writer(options);
            writer.setInput(&reader);
            writer.prepare(ctx);
            writer.execute(ctx);


        }

    }

