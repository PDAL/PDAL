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

            // create the reader
            LasReader reader = new LasReader("../../test/data/1.2-with-color.las");
            reader.initialize();

            // how many points do we have?
            ulong numPoints = reader.getNumPoints();
            Debug.Assert(numPoints == 1065);

            Bounds_double bounds = reader.getBounds();
            Debug.Assert(closeTo(bounds.getMinimum().get(0), 635619.85));
            Debug.Assert(closeTo(bounds.getMinimum().get(1), 848899.70000000007));
            Debug.Assert(closeTo(bounds.getMinimum().get(2), 406.59000000000003));
            Debug.Assert(closeTo(bounds.getMaximum().get(0), 638982.55));
            Debug.Assert(closeTo(bounds.getMaximum().get(1), 853535.43));
            Debug.Assert(closeTo(bounds.getMaximum().get(2), 586.38));

            // create the point buffer we'll read into
            // make it only hold 128 points a time, so we can show iterating
            Schema schema = reader.getSchema();
            PointBuffer data = new PointBuffer(schema, 128);

            // get the dimensions (fields) of the point record for the X, Y, and Z values
            int offsetX = schema.getDimensionIndex(DimensionId.Id.X_i32);
            int offsetY = schema.getDimensionIndex(DimensionId.Id.Y_i32);
            int offsetZ = schema.getDimensionIndex(DimensionId.Id.Z_i32);
            Dimension dimensionX = schema.getDimension((uint)offsetX);
            Dimension dimensionY = schema.getDimension((uint)offsetY);
            Dimension dimensionZ = schema.getDimension((uint)offsetZ);

            // make the iterator to read from the file
            StageSequentialIterator iter = reader.createSequentialIterator();

            uint totalRead = 0;

            while (!iter.atEnd())
            {
                uint numRead = iter.read(data);
                totalRead += numRead;

                Console.WriteLine(numRead + " points read this time");

                // did we just read the first block of 128 points?
                // if so, let's check the first point in the buffer
                // (which is point number 0 in the file) and make sure 
                // it is correct
                if (iter.getIndex() == 128)
                {
                    uint pointIndex = 0;

                    // get the raw data from the 1st point record in the point buffer
                    int x0raw = data.getField_Int32(pointIndex, offsetX);
                    int y0raw = data.getField_Int32(pointIndex, offsetY);
                    int z0raw = data.getField_Int32(pointIndex, offsetZ);

                    // LAS stores the data in scaled integer form: undo the scaling
                    // so we can see the real values as doubles
                    double x0 = dimensionX.applyScaling_Int32(x0raw);
                    double y0 = dimensionY.applyScaling_Int32(y0raw);
                    double z0 = dimensionZ.applyScaling_Int32(z0raw);

                    // make sure the X, Y, Z fields are correct!
                    Debug.Assert(x0 == 637012.240000);
                    Debug.Assert(y0 == 849028.310000);
                    Debug.Assert(z0 == 431.660000);
                    Console.WriteLine("point 0 is correct");
                }
            }

            // make sure we have read all the points in the file
            Debug.Assert(totalRead == numPoints);

            return;
        }
    }
}
