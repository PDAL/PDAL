#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"
#include <io/LasReader.hpp>
#include "../io/ArrowWriter.hpp"


namespace pdal
{
namespace arrow
{

TEST(ArrowWriterTest, write_array)
{

    Options readerOps;
    readerOps.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", Support::temppath("simple.feather"));
    ArrowWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    PointViewSet viewSet = writer.execute(table);

}

} // namespace arrow
} // namespace pdal

