import pdal
from pdal import libpdalpython
import unittest
import os

class TestPDALArray(unittest.TestCase):
  DATADIRECTORY = "../test"

  def fetch_xml(self, filename):
    import os
    DATADIRECTORY = os.environ.get('PDAL_TEST_DIR')
    if not DATADIRECTORY:
        DATADIRECTORY = self.DATADIRECTORY
    fn = DATADIRECTORY + os.path.sep +  filename
    output = ''
    with open(fn, 'rb') as f:
        output = f.read().decode('UTF-8')
    return output

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, '/data/pipeline/pipeline_read.xml')),
                       "missing test data")
  def test_construction(self):
    """Can we construct a PDAL pipeline"""
    xml = self.fetch_xml('/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, '/data/pipeline/pipeline_read.xml')),
                       "missing test data")
  def test_execution(self):
    """Can we execute a PDAL pipeline"""
    xml = self.fetch_xml('/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    self.assertGreater(len(r.xml), 1000)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, '/data/pipeline/pipeline_read.xml')),
                       "missing test data")
  def test_array(self):
    """Can we fetch PDAL data as a numpy array"""
    xml = self.fetch_xml('/data/pipeline/pipeline_read.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    arrays = r.arrays()
    self.assertEqual(len(arrays), 1)

    a = arrays[0]
    self.assertAlmostEqual(a[0][0], 637012.24, 7)
    self.assertAlmostEqual(a[1064][2], 423.92, 7)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, '/data/filters/chip.xml')),
                       "missing test data")
  def test_merged_arrays(self):
    """Can we fetch merged PDAL data """
    xml = self.fetch_xml('/data/filters/chip.xml')
    r = libpdalpython.PyPipeline(xml)
    r.execute()
    arrays = r.arrays()
    self.assertEqual(len(arrays), 43)
    # data are going to all be a little different
    # due to sorting not being stable
#     for a in arrays:
#         self.assertAlmostEqual(a[0][0], 494057.30, 2)
#         self.assertAlmostEqual(a[0][2], 130.63, 2)
def test_suite():
    return unittest.TestSuite(
        [TestPDALArray])

if __name__ == '__main__':
    unittest.main()
