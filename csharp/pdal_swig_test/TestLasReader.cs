using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Pdal;
using System.Diagnostics;

namespace pdal_swig_test
{
    internal class TestLasReader : TestBase
    {
        public TestLasReader()
        {
            Test1();
        }

        private void Test1()
        {
            Console.WriteLine("Starting LasReader test");

            LasReader reader = new LasReader("../../test/data/1.2-with-color.las");
            reader.initialize();

            ulong numPoints = reader.getNumPoints();

            Schema schema = reader.getSchema();
            SchemaLayout layout = new SchemaLayout(schema);

            PointBuffer data = new PointBuffer(layout, 512);

            StageSequentialIterator iter = reader.createSequentialIterator();

            uint numRead = iter.read(data);

            {
                Console.WriteLine("checking point 0...");
                uint index = 0;

                int offsetX = schema.getDimensionIndex(Dimension.Field.Field_X, Dimension.DataType.Int32);
                int offsetY = schema.getDimensionIndex(Dimension.Field.Field_Y, Dimension.DataType.Int32);
                int offsetZ = schema.getDimensionIndex(Dimension.Field.Field_Z, Dimension.DataType.Int32);

                int x0raw = data.getField_Int32(index, offsetX);
                int y0raw = data.getField_Int32(index, offsetY);
                int z0raw = data.getField_Int32(index, offsetZ);
                double x0 = schema.getDimension((uint)offsetX).applyScaling_Int32(x0raw);
                double y0 = schema.getDimension((uint)offsetY).applyScaling_Int32(y0raw);
                double z0 = schema.getDimension((uint)offsetZ).applyScaling_Int32(z0raw);

                Debug.Assert(x0 == 637012.240000);
                Debug.Assert(y0 == 849028.310000);
                Debug.Assert(z0 == 431.660000);
                Console.WriteLine("point 0 okay");
            }

            return;
        }
    }
}
