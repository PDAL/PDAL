import pdal
from pdal import libpdalpython
import unittest
import os

DATADIRECTORY = os.environ.get('PDAL_TEST_DIR')
if not DATADIRECTORY:
    DATADIRECTORY = "../test"

class TestPDALArray(unittest.TestCase):

  DATADIRECTORY = os.environ.get('PDAL_TEST_DIR')
  if not DATADIRECTORY:
      DATADIRECTORY = "../test"

  def fetch_json(self, filename):
    import os
    fn = DATADIRECTORY + os.path.sep +  filename
    output = ''
    with open(fn, 'rb') as f:
        output = f.read().decode('UTF-8')
    return output

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/pipeline_read.json')),
                       os.path.join(DATADIRECTORY, 'data/pipeline/pipeline_read.json'))
  def test_construction(self):
    """Can we construct a PDAL pipeline"""
    json = self.fetch_json('/data/pipeline/pipeline_read.json')
    r = libpdalpython.PyPipeline(json)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/pipeline_read.json')),
                       "missing test data")
  def test_execution(self):
    """Can we execute a PDAL pipeline"""
    x = self.fetch_json('/data/pipeline/pipeline_read.json')
    r = libpdalpython.PyPipeline(x)
    r.execute()
    import sys
    self.assertGreater(len(r.json), 200)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/pipeline/pipeline_read.json')),
                       "missing test data")
  def test_array(self):
    """Can we fetch PDAL data as a numpy array"""
    json = self.fetch_json('/data/pipeline/pipeline_read.json')
    r = libpdalpython.PyPipeline(json)
    r.execute()
    arrays = r.arrays()
    self.assertEqual(len(arrays), 1)

    a = arrays[0]
    self.assertAlmostEqual(a[0][0], 637012.24, 7)
    self.assertAlmostEqual(a[1064][2], 423.92, 7)

  @unittest.skipUnless(os.path.exists(os.path.join(DATADIRECTORY, 'data/filters/chip.json')),
                       "missing test data")
  def test_merged_arrays(self):
    """Can we fetch merged PDAL data """
    json = self.fetch_json('/data/filters/chip.json')
    r = libpdalpython.PyPipeline(json)
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
