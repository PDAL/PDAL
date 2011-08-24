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
            //istream stream = Utils.openFile("../../test/data/1.2-with-color.las");
            
            LasReader reader = new LasReader();

            ulong numPoints = reader.getNumPoints();

            Schema schema = reader.getHeader().getSchema();
            SchemaLayout layout = new SchemaLayout(schema);

            PointData data = new PointData(layout, 1);

            uint numRead = reader.read(data);

            {
                uint offsetX = (uint)schema.getDimensionIndex(Dimension.Field.Field_X);
                uint offsetY = (uint)schema.getDimensionIndex(Dimension.Field.Field_Y);
                uint offsetZ = (uint)schema.getDimensionIndex(Dimension.Field.Field_Z);

                uint index = 0;
                Int32 x0raw = data.getField_Int32(index, offsetX);
                Int32 y0raw = data.getField_Int32(index, offsetY);
                Int32 z0raw = data.getField_Int32(index, offsetZ);
                double x0 = schema.getDimension(offsetX).getNumericValue_Int32(x0raw);
                double y0 = schema.getDimension(offsetY).getNumericValue_Int32(y0raw);
                double z0 = schema.getDimension(offsetZ).getNumericValue_Int32(z0raw);

                Debug.Assert(x0 == 637012.240000);
                Debug.Assert(y0 == 849028.310000);
                Debug.Assert(z0 == 431.660000);
            }

            //Utils.closeFile(stream);

            return;
        }
    }
}
