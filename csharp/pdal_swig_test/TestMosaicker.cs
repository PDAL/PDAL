using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Pdal;
using System.Diagnostics;

namespace pdal_swig_test
{
    internal class TestMosaicker : TestBase
    {
        public TestMosaicker()
        {
            Test1();
        }

        private void Test1()
        {
            Console.WriteLine("Starting Mosaicker test");

            // create the reader
            LasReader reader1 = new LasReader("../../test/data/1.2-with-color.las");
            LasReader reader2 = new LasReader("../../test/data/1.2-with-color.las");

            std_vector_Stage stages = new std_vector_Stage();
            stages.Add(reader1);
            stages.Add(reader2);

            MosaicFilter filter = new MosaicFilter(stages, Options.none());

            filter.initialize();

            // how many points do we have?
            ulong numPoints = filter.getNumPoints();
            Debug.Assert(numPoints == 1065 * 2);

            return;
        }
    }
}
