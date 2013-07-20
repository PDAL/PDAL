#include <pdal/Schema.hpp>
#include <pdal/PointBuffer.hpp>

#include "timer.hpp"

#if defined(PDAL_COMPILER_MSVC)
#   define PDAL_NOINLINE __declspec(noinline)
#else
//  For gcc, clang, icc
#   define PDAL_NOINLINE __attribute__((noinline))
#endif


using namespace pdal;

// Various wrapper functions around schema.getDimension*().  These exist to
// avoid putting the getDimension() calls directly inside a loop, which
// wouldn't quite accurately represent a real use case - the compiler may be
// able to optimize some things out if these weren't declared noinline.


// Lookup dimension in schema, forcing the creation of two boost::string_refs
// from const char* (a couple of extra strlen() calls per lookup)
//
PDAL_NOINLINE const Dimension* lookup(Schema& schema, const char* name,
                                      const char* namespc)
{
    return schema.getDimensionPtr(name, namespc);
}


// Lookup dimension, inefficiently swallowing exception if not found
PDAL_NOINLINE const Dimension* lookupEx(Schema& schema,
                                        const char* name,
                                        const char* namespc)
{
    try
    {
        const Dimension& d = schema.getDimension(name, namespc);
        return &d;
    }
    catch(pdal::dimension_not_found&)
    {
        return 0;
    }
}


// Lookup dimensions, forcing the allocation of a pair of std::string instances
// each time.
PDAL_NOINLINE const Dimension* lookupStdStr(Schema& schema, const std::string& name,
                                            const std::string& namespc)
{
    return schema.getDimensionPtr(name, namespc);
}


Dimension makeDimension(const std::string& name,
                        dimension::Interpretation interpretation,
                        dimension::size_type sizeInBytes,
                        const std::string& namespc)
{
    Dimension d(name, interpretation, sizeInBytes);
    d.setNamespace(namespc);
    return d;
}


int main(int argc, char* argv[])
{
    Schema schema;
    {
        // Add representative set of standard las dimensions
        std::string ns = "driver.las.reader";
        schema.appendDimension(makeDimension("X", dimension::SignedInteger, 4, ns));
        schema.appendDimension(makeDimension("Y", dimension::SignedInteger, 4, ns));
        schema.appendDimension(makeDimension("Z", dimension::SignedInteger, 4, ns));
        schema.appendDimension(makeDimension("Time", dimension::Float, 8, ns));
        schema.appendDimension(makeDimension("Intensity", dimension::UnsignedInteger, 2, ns));
        schema.appendDimension(makeDimension("ReturnNumber", dimension::UnsignedInteger, 1, ns));
        schema.appendDimension(makeDimension("NumberOfReturns", dimension::UnsignedInteger, 1, ns));
        schema.appendDimension(makeDimension("ScanDirectionFlag", dimension::UnsignedInteger, 1, ns));
        schema.appendDimension(makeDimension("EdgeOfFlightLine", dimension::UnsignedInteger, 1, ns));
        schema.appendDimension(makeDimension("Classification", dimension::UnsignedInteger, 1, ns));
        schema.appendDimension(makeDimension("ScanAngleRank", dimension::SignedInteger, 1, ns));
        schema.appendDimension(makeDimension("PointSourceId", dimension::UnsignedInteger, 2, ns));
    }

    // Lookup without namespace
    {
        size_t niter = 10000000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookup(schema, "Intensity", "");
        std::cout << "Time per lookup of \"Intensity\" without namespace: "
                  << 1e9 * timer()/niter << " ns\n";
    }

    // Lookup with a namespace
    {
        size_t niter = 10000000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookup(schema, "Intensity", "driver.las.reader");
        std::cout << "Time per lookup of \"Intensity\" with namespace \"driver.las.reader\": "
                  << 1e9 * timer()/niter << " ns\n";
    }

    // Lookup with embedded namespace
    {
        size_t niter = 10000000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookup(schema, "driver.las.reader.Intensity", "");
        std::cout << "Time per lookup of \"driver.las.reader.Intensity\": "
                  << 1e9 * timer()/niter << " ns\n";
    }

    // Lookup of a missing dimension
    {
        size_t niter = 100000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookup(schema, "Intensity_", "driver.las.reader");
        std::cout << "Time per lookup of missing \"Intensity_\" with namespace \"driver.las.reader\": "
                  << 1e9 * timer()/niter << " ns\n";
    }


    // Test speed hit for doing several dubious things.

    // Throwing and catching an exception per lookup
    {
        size_t niter = 100000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookupEx(schema, "Intensity_", "driver.las.reader");
        std::cout << "Time per THROWING lookup of missing \"Intensity_\" with namespace \"driver.las.reader\": "
                  << 1e9 * timer()/niter << " ns\n";
    }

    // Forcing the allocation of a pair of std::string per lookup
    {
        size_t niter = 10000000;
        Timer timer;
        for (size_t i = 0; i < niter; ++i)
            lookupStdStr(schema, "Intensity", "driver.las.reader");
        std::cout << "Time per lookup of \"Intensity\" with namespace \"driver.las.reader\" including std::string creation: "
                  << 1e9 * timer()/niter << " ns\n";
    }

    // TODO:
    // * lookup of inheriting dimensions

    return 0;
}

