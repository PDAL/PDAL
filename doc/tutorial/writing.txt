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

    g++ pdal-tutorial.cpp -o kp -Iinclude/ -Lbin/ -l pdalcpp -g

.. note::

    Refer to :ref:`building` for information on how to build PDAL.
    

Includes
-------------------------------------------------------------------------------

As of PDAL 1.0.0, there's a couple of include files available for use.  The 
main one, `<pdal/pdal.hpp>`, brings in most of the utility and basic classes 
used throughout the PDAL system. Additionally, the following includes are 
available subject to having the required include directories pathed:

* `#include <pdal/Drivers.hpp>`
* `#include <pdal/Filters.hpp>`
* `#include <pdal/Kernel.hpp>`


.. note::

    Drivers, Filters, and Kernel bring in *all* of the respective sub includes 
    for those sections of the source tree. You are not required to include 
    everything if you don't need it, however, and it is still possible to 
    selectively include the classes you need as shown in the example below.



.. code-block:: cpp


    #include <pdal/pdal.hpp>
    #include <pdal/drivers/buffer/Reader.hpp>
    #include <pdal/filters/Scaling.hpp>
    #include <pdal/drivers/las/Writer.hpp>

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

    pdal::PointBuffer* createBuffer()
    {

        pdal::Schema schema;
    
        pdal::Dimension x("X", pdal::dimension::Float, 8);
        pdal::Dimension y("Y", pdal::dimension::Float, 8);
        pdal::Dimension z("Z", pdal::dimension::Float, 8);
    
        x.setNamespace("myvector");
        y.setNamespace("myvector");
        z.setNamespace("myvector");
        x.createUUID(); y.createUUID(); z.createUUID();
    
        schema.appendDimension(x); 
        schema.appendDimension(y); 
        schema.appendDimension(z);
    
        // 1000 is the capacity of the PointBuffer you wish to have
        pdal::PointBuffer* output = new pdal::PointBuffer(schema, 1000); 
    
        return output;
    }


    void fillBuffer(pdal::PointBuffer* buffer, std::vector<Point> const& data)
    {
        // You must fetch the buffer's schema & and get a reference 
        // to the X dimension. Some operation may have updated the 
        // buffer's schema 
        pdal::Dimension const& x = buffer->getSchema().getDimension("X");
        pdal::Dimension const& y = buffer->getSchema().getDimension("Y");
        pdal::Dimension const& z = buffer->getSchema().getDimension("Z");
    
        for (int i = 0; i < data.size(); ++i)
        {
            // You must set the "raw" xyz values that have 
            // been descaled by the setNumericOffset and setNumericScale 
            // you provided when you created the dimensions and added 
            // them to the schema
            Point const& pt = data[i];
            buffer->setField<double>(x, i, pt.x); 
            buffer->setField<double>(y, i, pt.y); 
            buffer->setField<double>(z, i, pt.z); 
        }
    
        buffer->setNumPoints(data.size());
    }

    pdal::Options getOptions()
    {
    
        pdal::Options opts;
    
        pdal::Option scalex("scale", 0.001f, "fpscale");
        pdal::Option offsetx("offset", 0.0f, "offset");
        pdal::Option xdim("dimension", "X", "dimension to scale");
        pdal::Option sizex("size", 4, "size");
        pdal::Option typex("type", "SignedInteger", "tye");
    
        pdal::Options xs;
        xs.add(scalex);
        xs.add(offsetx);
        xs.add(sizex);
        xs.add(typex);

        xdim.setOptions(xs);
        opts.add(xdim);

        pdal::Option scaley("scale", 0.001f, "fpscale");
        pdal::Option offsety("offset", 0.0f, "offset");
        pdal::Option sizey("size", 4, "size");
        pdal::Option typey("type", "SignedInteger", "tye");
        pdal::Option ydim("dimension", "Y", "dimension to scale");
        pdal::Options ys;
        ys.add(scaley);
        ys.add(offsety);
        ys.add(sizey);
        ys.add(typey);
        ydim.setOptions(ys);
        opts.add(ydim);

        pdal::Option scalez("scale", 0.01f, "fpscale");
        pdal::Option offsetz("offset", 0, "offset");
        pdal::Option sizez("size", 4, "size");
        pdal::Option typez("type", "SignedInteger", "tye");
        pdal::Option zdim("dimension", "Z", "dimension to scale");
        pdal::Options zs;
        zs.add(scalez);
        zs.add(offsetz);
        zs.add(sizez);
        zs.add(typez);
        zdim.setOptions(zs);
        opts.add(zdim);
    
        return opts;

    }

    int main( int argc, const char* argv[] )
    {

    
        // No options are used for PointBuffer reader
        pdal::Options options = getOptions();

        pdal::Option debug("debug", true, "");
        pdal::Option verbose("verbose", 7, "");
        // options.add(debug);
        // options.add(verbose);    
    
        pdal::Option filename("filename", "myfile.las");
        options.add(filename);
    
        {
            pdal::PointBuffer* buffer = createBuffer();
    
            std::vector<Point> data = getMyData();
        
            fillBuffer(buffer, data);


            pdal::drivers::buffer::Reader reader(options, *buffer);
            pdal::filters::Scaling scaling(reader, options);
            pdal::drivers::las::Writer writer(scaling, options);


            writer.initialize();
            writer.write(buffer->getNumPoints());
        }
    
    }

