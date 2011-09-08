using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Pdal;
using System.Diagnostics;

namespace pdal_swig_test
{
    internal class TestLasWriter : TestBase
    {
        public TestLasWriter()
        {
            Test1();
        }

        private void Test1()
        {
            Console.WriteLine("Starting LasWriter test1");

            {
                // create the reader
                Options readerOpts = new Options();
                Option readerOpt1 = new Option();
                readerOpt1.setValue_String("../../test/data/1.2-with-color.las");
                readerOpt1.setName("filename");

                readerOpts.add(readerOpt1);
                LasReader reader = new LasReader(readerOpts);

                Options writerOpts = new Options();
                Option writerOpt1 = new Option();
                writerOpt1.setValue_String("foo.laz");
                writerOpt1.setName("filename");
                writerOpts.add(writerOpt1);
                LasWriter writer = new LasWriter(reader, writerOpts);
                writer.initialize();

                writer.setCompressed(true);
                writer.setDate(0, 0);
                writer.setPointFormat(PointFormat.PointFormat3);
                writer.setSystemIdentifier("");
                writer.setGeneratingSoftware("TerraScan");

                // how many points do we have?
                ulong numPoints = reader.getNumPoints();
                Debug.Assert(numPoints == 1065);

                ulong numWritten = writer.write(numPoints);
                Debug.Assert(numWritten == 1065);
            }

            Console.WriteLine("checking output...");

            {
                Options opts = new Options();
                Option opt = new Option();
                opt.setValue_String("foo.laz");
                opt.setName("filename");
                opts.add(opt);

                LasReader reader = new LasReader(opts);
                reader.initialize();

                Debug.Assert(reader.isCompressed() == true);

                ulong numPoints = reader.getNumPoints();
                Debug.Assert(numPoints == 1065);
            }

            Console.WriteLine("done!");

            return;
        }

    }
}
