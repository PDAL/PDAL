using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Libpc;
using System.Diagnostics;

namespace libpc_swig_test
{
   internal class TestDimension : TestBase
   {
      public TestDimension()
      {
         Test1();
      }

      private void Test1()
      {
         string s = Dimension.getDataTypeName(Dimension.DataType.Double);
         Assert(s == "Double");

         Dimension d = new Dimension(Dimension.Field.Field_Blue, Dimension.DataType.Float);

         return;
      }
   }
}
